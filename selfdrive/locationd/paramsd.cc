#include <future>
#include <iostream>
#include <cassert>
#include <csignal>
#include <unistd.h>

#include <capnp/serialize-packed.h>
#include "json11.hpp"

#include "common/swaglog.h"
#include "common/messaging.h"
#include "common/params.h"
#include "common/timing.h"

#include "messaging.hpp"
#include "locationd_yawrate.h"
#include "params_learner.h"

#include "common/util.h"

void sigpipe_handler(int sig) {
  LOGE("SIGPIPE received");
}


int main(int argc, char *argv[]) {
  signal(SIGPIPE, (sighandler_t)sigpipe_handler);

  SubMaster sm({"controlsState", "sensorEvents", "cameraOdometry", "pathPlan"});
  PubMaster pm({"liveParameters"});

  Localizer localizer;

  // Read car params
  char *value;
  size_t value_sz = 0;

  LOGW("waiting for params to set vehicle model");
  while (true) {
    read_db_value("CarParams", &value, &value_sz);
    if (value_sz > 0) break;
    usleep(100*1000);
  }
  LOGW("got %d bytes CarParams", value_sz);

  // make copy due to alignment issues
  auto amsg = kj::heapArray<capnp::word>((value_sz / sizeof(capnp::word)) + 1);
  memcpy(amsg.begin(), value, value_sz);
  free(value);

  capnp::FlatArrayMessageReader cmsg(amsg);
  cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

  // Read params from previous run
  const int result = read_db_value("LiveParameters", &value, &value_sz);

  std::string fingerprint = car_params.getCarFingerprint();
  std::string vin = car_params.getCarVin();
  double sR = car_params.getSteerRatio();
  double x = 1.0;
  double ao = 0.0;
  double posenet_invalid_count = 0;

  auto lateralsRatom = car_params.getLateralsRatom();
  int  carParams_learnerParams = lateralsRatom.getLearnerParams();  

  if (result == 0){
    auto str = std::string(value, value_sz);
    free(value);

    std::string err;
    auto json = json11::Json::parse(str, err);
    if (json.is_null() || !err.empty()) {
      std::string log = "Error parsing json: " + err;
      LOGW(log.c_str());
    } else {
      std::string new_fingerprint = json["carFingerprint"].string_value();
      std::string new_vin = json["carVin"].string_value();

      if (fingerprint == new_fingerprint && vin == new_vin) {
        std::string log = "Parameter starting with: " + str;
        LOGW(log.c_str());

        if( carParams_learnerParams )
           sR = json["steerRatio"].number_value();

        x = json["stiffnessFactor"].number_value();
        ao = json["angleOffsetAverage"].number_value();
      }
    }
  }

  ParamsLearner learner(car_params, ao, x, sR, 1.0);


  // Main loop
  int save_counter = 0;
  while (true){
    if (sm.update(100) == 0) continue;

    if ( !carParams_learnerParams && sm.updated("pathPlan") )
    {
      auto data = sm["pathPlan"].getPathPlan();
      learner.sR = data.getSteerRatio();
    }

    if ( sm.updated("controlsState") )
    {
      localizer.handle_log(sm["controlsState"]);
      save_counter++;

      double yaw_rate = -localizer.x[0];
      bool valid = learner.update(yaw_rate, localizer.car_speed, localizer.steering_angle);

      double angle_offset_degrees = RADIANS_TO_DEGREES * learner.ao;
      double angle_offset_average_degrees = RADIANS_TO_DEGREES * learner.slow_ao;

      capnp::MallocMessageBuilder msg;
      cereal::Event::Builder event = msg.initRoot<cereal::Event>();
      event.setLogMonoTime(nanos_since_boot());
      auto live_params = event.initLiveParameters();
      live_params.setValid(valid);
      live_params.setYawRate(localizer.x[0]);
      live_params.setGyroBias(localizer.x[1]);
      live_params.setAngleOffset(angle_offset_degrees);
      live_params.setAngleOffsetAverage(angle_offset_average_degrees);
      live_params.setStiffnessFactor(learner.x);
      live_params.setSteerRatio(learner.sR);

      pm.send("liveParameters", msg);

      // Save parameters every minute
      if (save_counter % 6000 == 0) {
        json11::Json json = json11::Json::object {
                                                  {"carVin", vin},
                                                  {"carFingerprint", fingerprint},
                                                  {"steerRatio", learner.sR},
                                                  {"stiffnessFactor", learner.x},
                                                  {"angleOffsetAverage", angle_offset_average_degrees},
        };

        std::string out = json.dump();
        std::async(std::launch::async,
                    [out]{
                      write_db_value("LiveParameters", out.c_str(), out.length());
                    });
      }
    }
    if (sm.updated("sensorEvents")){
      localizer.handle_log(sm["sensorEvents"]);
    }
    if (sm.updated("cameraOdometry")){
      localizer.handle_log(sm["cameraOdometry"]);
    } 
  }
  return 0;
}
