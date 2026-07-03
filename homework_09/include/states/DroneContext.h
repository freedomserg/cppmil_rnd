#pragma once

#include "../Types.h"

struct DroneContext {
      const DroneConfig& cfg;        // read-only
      float acceleration = 0.0f;     // precomputed in init()
      
      // planning inputs — set each step before execute()
      float desiredDir = 0.0f;
      float deltaAngle = 0.0f;
      
      // kinematic state — persists across steps
      Coord pos       = {};
      float direction = 0.0f;
      float speed     = 0.0f;
      
      // per-step output
      float deltaPath = 0.0f;
  };