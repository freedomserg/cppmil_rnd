#define _USE_MATH_DEFINES
#define ENABLE_LOG   1
#define ENABLE_DEBUG 1

#if ENABLE_LOG
#  define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#  define LOG(msg)
#endif
#if ENABLE_DEBUG
#  define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#  define DEBUG(msg)
#endif

#include <iostream>
#include <fstream>
#include <cstring>
#include <cmath>
#include "json.hpp"

using json = nlohmann::json;

constexpr float GRAVITY               = 9.81f;
constexpr int   MAX_STEPS             = 10000;
constexpr int   NUM_APPROX_STEPS      = 3;
constexpr float TIME_CHANGE_THRESHOLD = 0.05f;

// ============================================================
// Data structures
// ============================================================

struct Coord {
    float x{};
    float y{};

    Coord operator+(const Coord& o) const { return {x + o.x, y + o.y}; }
    Coord operator-(const Coord& o) const { return {x - o.x, y - o.y}; }
    Coord operator*(float s)        const { return {x * s,   y * s};   }

    Coord operator/(float s) const {
        if (s < 1e-6f) {
            std::cerr << "Error: division by zero in Coord operator/" << std::endl;
            return {0.0f, 0.0f};
        }
        return {x / s, y / s};
    }

    bool operator==(const Coord& o) const {
        return std::fabs(x - o.x) < 1e-6f && std::fabs(y - o.y) < 1e-6f;
    }

    float length() const { return std::hypot(x, y); }

    Coord normalize() const {
        float len = length();
        if (len < 1e-6f) {
            std::cerr << "Error: Coord::normalize called on zero-length vector" << std::endl;
            return {0.0f, 0.0f};
        }
        return *this / len;
    }
};

struct AmmoParams {
    char  name[32]{};
    float mass{};
    float drag{};
    float lift{};
};

struct DroneConfig {
    Coord initialPos{};
    float altitude{};
    float initialDir{};
    float attackSpeed{};
    float accelPath{};
    char  ammoName[32]{};
    float arrayTimeStep{};
    float simTimeStep{};
    float hitRadius{};
    float angularSpeed{};
    float turnThreshold{};
};

struct SimStep {
    Coord pos{};
    float direction{};
    int   state{};
    int   targetIdx{};
    Coord dropPoint{};
    Coord aimPoint{};
    Coord predictedTarget{};
};

enum DroneState { STOPPED, ACCELERATING, DECELERATING, TURNING, MOVING };

struct Target {
    const Coord* positions;
    int          timeStepsCount;
};

// ============================================================
// Interfaces
// ============================================================

class ITargetProvider {
public:
    virtual int    getTargetCount()   = 0;
    virtual Target getTarget(int idx) = 0;
    virtual ~ITargetProvider() {}
};

class IBallisticSolver {
public:
    // Computes fire point and total flight time for the given drone state and target track
    // alreadyApproaching: drone is already in route to this target — skip intermediate maneuver
    // outPredictedTarget: where the target is expected to be at drop time
    // outAimPoint: current drone aim point if to drop right now
    virtual bool solve(
        const Coord&       dronePos,
        float              droneDir,
        float              droneSpeed,
        const Target&      target,
        float              currentTime,
        bool               alreadyApproaching,
        const DroneConfig& cfg,
        const AmmoParams&  ammo,
        Coord& outFirePos,
        Coord& outIntermediatePos,
        bool&  outHasIntermediate,
        float& outTotalTime,
        Coord& outPredictedTarget,
        Coord& outAimPoint) = 0;

    virtual ~IBallisticSolver() {}
};

class IConfigLoader {
public:
    virtual bool        load()          = 0;
    virtual DroneConfig getConfig()     const = 0;
    virtual AmmoParams  getAmmoParams() const = 0;
    virtual ~IConfigLoader() {}
};

// ============================================================
// Utilities
// ============================================================

// приводить кут до діапазону [-π, π]
inline float normalizeAngle(float a) {
    while (a >  M_PI) a -= 2.0f * (float)M_PI;
    while (a < -M_PI) a += 2.0f * (float)M_PI;
    return a;
}

// a = v² / (2·s)
inline float calcAccel(float speed, float accelPath) {
    return speed * speed / (2.0f * accelPath);
}

inline float calcDistance(const Coord& a, const Coord& b) {
    return (b - a).length();
}

// s = v² / (2·a)
inline float accelPathFromSpeed(float v, float a) {
    if (a < 1e-6f) { std::cerr << "Error: a must be > 0 in accelPathFromSpeed" << std::endl; return 0.0f; }
    return v * v / (2.0f * a);
}

// t = sqrt(2·d / a)
inline float accelTimeFromDist(float d, float a) {
    if (a < 1e-6f) { std::cerr << "Error: a must be > 0 in accelTimeFromDist" << std::endl; return 0.0f; }
    return std::sqrt(2.0f * d / a);
}

// ============================================================
// File I/O
// ============================================================

bool writeOutputToFile(const char* filename, const SimStep* simSteps, int n) {
    std::fstream f(filename, std::ios::out);
    if (!f) { LOG("Error opening output file: " << filename); return false; }

    json out;
    out["totalSteps"] = n;
    out["steps"]      = json::array();
    for (int i = 0; i < n; ++i) {
        json s;
        s["position"]        = {{"x", simSteps[i].pos.x},            {"y", simSteps[i].pos.y}};
        s["direction"]       = simSteps[i].direction;
        s["state"]           = simSteps[i].state;
        s["targetIndex"]     = simSteps[i].targetIdx;
        s["dropPoint"]       = {{"x", simSteps[i].dropPoint.x},       {"y", simSteps[i].dropPoint.y}};
        s["aimPoint"]        = {{"x", simSteps[i].aimPoint.x},        {"y", simSteps[i].aimPoint.y}};
        s["predictedTarget"] = {{"x", simSteps[i].predictedTarget.x}, {"y", simSteps[i].predictedTarget.y}};
        out["steps"].push_back(s);
    }
    f << out.dump(2);
    f.close();
    return true;
}

// ============================================================
// Implementations
// ============================================================

class FileConfigLoader : public IConfigLoader {
    const char* configFile_;
    const char* ammoFile_;
    DroneConfig cfg_{};
    AmmoParams  ammo_{};

    DroneConfig loadDroneConfig(const char* filename, bool& outLoaded) {
        std::fstream f(filename, std::ios::in);
        if (!f) { LOG("Error opening input file: " << filename); outLoaded = false; return {}; }

        json data = json::parse(f);
        f.close();

        DroneConfig cfg = {
            .initialPos    = {data["drone"]["position"]["x"].get<float>(),
                              data["drone"]["position"]["y"].get<float>()},
            .altitude      = data["drone"]["altitude"].get<float>(),
            .initialDir    = data["drone"]["initialDirection"].get<float>(),
            .attackSpeed   = data["drone"]["attackSpeed"].get<float>(),
            .accelPath     = data["drone"]["accelerationPath"].get<float>(),
            .arrayTimeStep = data["targetArrayTimeStep"].get<float>(),
            .simTimeStep   = data["simulation"]["timeStep"].get<float>(),
            .hitRadius     = data["simulation"]["hitRadius"].get<float>(),
            .angularSpeed  = data["drone"]["angularSpeed"].get<float>(),
            .turnThreshold = data["drone"]["turnThreshold"].get<float>()
        };
        std::strncpy(cfg.ammoName, data["ammo"].get<std::string>().c_str(), sizeof(cfg.ammoName) - 1);
        cfg.ammoName[sizeof(cfg.ammoName) - 1] = '\0';
        outLoaded = true;
        return cfg;
    }

    AmmoParams* loadAmmoParamsArray(const char* filename, int& outAmmoCount) {
        std::fstream f(filename, std::ios::in);
        if (!f) { LOG("Error opening ammo parameters file: " << filename); outAmmoCount = 0; return nullptr; }

        json data = json::parse(f);
        f.close();

        int n = data.size();
        AmmoParams* arr = new AmmoParams[n];
        for (int i = 0; i < n; ++i) {
            arr[i] = { .mass = data[i]["mass"].get<float>(),
                       .drag = data[i]["drag"].get<float>(),
                       .lift = data[i]["lift"].get<float>() };
            std::strncpy(arr[i].name, data[i]["name"].get<std::string>().c_str(), sizeof(arr[i].name) - 1);
            arr[i].name[sizeof(arr[i].name) - 1] = '\0';
        }
        outAmmoCount = n;
        return arr;
    }

public:
    FileConfigLoader(const char* configFile, const char* ammoFile)
        : configFile_(configFile), ammoFile_(ammoFile) {}

    bool load() override {
        bool ok = false;
        cfg_ = loadDroneConfig(configFile_, ok);
        if (!ok) return false;

        int n = 0;
        AmmoParams* arr = loadAmmoParamsArray(ammoFile_, n);
        if (!arr) return false;

        bool found = false;
        for (int i = 0; i < n; ++i) {
            if (std::strcmp(cfg_.ammoName, arr[i].name) == 0) {
                ammo_  = arr[i];
                found  = true;
                break;
            }
        }
        delete[] arr;
        if (!found) { LOG("Error: unknown ammo type: " << cfg_.ammoName); return false; }

        if (std::fabs(cfg_.initialDir)    > M_PI)  { LOG("Error: initialDir out of [-π,π]");     return false; }
        if (cfg_.attackSpeed   <= 0.0f) { LOG("Error: attackSpeed must be > 0");   return false; }
        if (cfg_.accelPath     <= 0.0f) { LOG("Error: accelPath must be > 0");     return false; }
        if (cfg_.arrayTimeStep <= 0.0f) { LOG("Error: arrayTimeStep must be > 0"); return false; }
        if (cfg_.simTimeStep   <= 0.0f) { LOG("Error: simTimeStep must be > 0");   return false; }
        if (cfg_.hitRadius     <= 0.0f) { LOG("Error: hitRadius must be > 0");     return false; }
        if (cfg_.angularSpeed  <= 0.0f) { LOG("Error: angularSpeed must be > 0");  return false; }
        if (std::fabs(cfg_.turnThreshold) > M_PI) { LOG("Error: turnThreshold out of [-π,π]"); return false; }

        return true;
    }

    DroneConfig getConfig()     const override { return cfg_; }
    AmmoParams  getAmmoParams() const override { return ammo_; }
};

class JsonTargetProvider : public ITargetProvider {
    Coord** arr_   = nullptr;
    int     count_ = 0;
    int     steps_ = 0;

    Coord** loadTargetsCoordinatesArray(const char* filename, int& outCount, int& outSteps) {
        std::fstream f(filename, std::ios::in);
        if (!f) { LOG("Error opening targets file: " << filename); outCount = outSteps = 0; return nullptr; }

        json data = json::parse(f);
        f.close();

        int count = data["targetCount"];
        int steps = data["timeSteps"];
        Coord** arr = new Coord*[count];
        for (int i = 0; i < count; ++i) {
            arr[i] = new Coord[steps];
            for (int j = 0; j < steps; ++j)
                arr[i][j] = {data["targets"][i]["positions"][j]["x"].get<float>(),
                             data["targets"][i]["positions"][j]["y"].get<float>()};
        }
        outCount = count;
        outSteps = steps;
        return arr;
    }

public:
    JsonTargetProvider(const char* filename) {
        arr_ = loadTargetsCoordinatesArray(filename, count_, steps_);
    }

    ~JsonTargetProvider() override {
        if (arr_) {
            for (int i = 0; i < count_; ++i) delete[] arr_[i];
            delete[] arr_;
        }
    }

    int    getTargetCount()   override { return count_; }
    Target getTarget(int idx) override { return {arr_[idx], steps_}; }
};

class AnalyticalSolver : public IBallisticSolver {
    float ammoFlightTime_ = 0.0f;
    float hDist_          = 0.0f;
    bool  ready_          = false;

    // Розраховує час падіння боєприпасу з висоти cfg.altitude до землі.
    // Рівняння руху — кубічне; вирішується методом Кардано через arccos.
    bool calcAmmoDropTime(const DroneConfig& cfg, const AmmoParams& ammo,
                                 float& outT) {
        float a = ammo.drag * GRAVITY * ammo.mass
                - 2.0f * ammo.drag * ammo.drag * ammo.lift * cfg.attackSpeed;
        if (std::fabs(a) < 1e-6f) { LOG("Error: coeff a ~ 0, a = " << a); return false; }

        float b = -3.0f * GRAVITY * ammo.mass * ammo.mass
                + 3.0f * ammo.drag * ammo.lift * ammo.mass * cfg.attackSpeed;
        float c = 6.0f * ammo.mass * ammo.mass * cfg.altitude;
        float p = -b * b / (3.0f * a * a);
        float q =  2.0f * b * b * b / (27.0f * a * a * a) + c / a;

        if (p >= 0) { LOG("Error: p must be negative, p = " << p); return false; }

        float arg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);
        if (arg < -1.0f - 1e-6f || arg > 1.0f + 1e-6f) {
            LOG("Error: arccos arg out of [-1,1]: " << arg); return false;
        }
        arg = std::max(-1.0f, std::min(1.0f, arg));
        float phi = std::acos(arg);

        // t = 2√(−p/3) · cos( (φ + 4π) / 3 ) − b / (3a)
        float t = 2.0f * std::sqrt(-p / 3.0f)
                * std::cos((phi + 4.0f * (float)M_PI) / 3.0f) - b / (3.0f * a);
        if (t < 0) { LOG("Error: negative flight time, t = " << t); return false; }

        outT = t;
        return true;
    }

    // Горизонтальна відстань яку пролетить боєприпас за час t.
    float calcAmmoHDistance(const DroneConfig& cfg, const AmmoParams& ammo, float t) {
        float V = cfg.attackSpeed, m = ammo.mass, d = ammo.drag, l = ammo.lift;
        float l2 = l*l, d2 = d*d, d3 = d2*d, d4 = d3*d;
        float m2 = m*m, m3 = m2*m, m4 = m3*m, lp = 1.0f + l2;
        return V*t
            - std::pow(t,2)*d*V/(2.0f*m)
            + std::pow(t,3)*(6.0f*d*GRAVITY*l*m - 6.0f*d2*(l2-1.0f)*V)/(36.0f*m2)
            + std::pow(t,4)*(-6.0f*d2*GRAVITY*l*(1.0f+l2+l2*l2)*m
                + 3.0f*d3*l2*lp*V + 6.0f*d3*l2*l2*lp*V)/(36.0f*lp*lp*m3)
            + std::pow(t,5)*(3.0f*d3*GRAVITY*l2*l*m - 3.0f*d4*l2*lp*V)/(36.0f*lp*m4);
    }

    // Визначає точку скиду та якщо потрібно проміжну точку розгону.
    void getIntermediateAndDropPoint(
        const Coord& dronePos, const Coord& targetPos,
        float hDist, float accelPath,
        Coord& outIntermediatePos, Coord& outFirePos, bool& outHasIntermediate)
    {
        Coord local = dronePos;
        float dist  = calcDistance(local, targetPos);

        outHasIntermediate = (hDist + accelPath > dist);
        if (outHasIntermediate) {
            if (dist < 1e-6f)
                local = {targetPos.x - (hDist + accelPath), targetPos.y};
            else
                local = targetPos - (targetPos - local) * ((hDist + accelPath) / dist);
            dist = calcDistance(local, targetPos);
        }
        outIntermediatePos = local;
        outFirePos = targetPos - (targetPos - local).normalize() * hDist;
    }

    // Знаходить точну позицію цілі в момент часу t методом лінійної інтерполяції.
    // Масив є циклічним — після останньої точки знову йде перша.
    Coord interpolate(const Target& target, float t, float arrayTimeStep) {
        int          raw  = static_cast<int>(std::floor(t / arrayTimeStep));
        int          n    = target.timeStepsCount;
        int          idx  = raw % n;
        int          next = (idx + 1) % n;
        float        frac = (t - raw * arrayTimeStep) / arrayTimeStep;
        const Coord* pos  = target.positions;
        return {pos[idx].x + (pos[next].x - pos[idx].x) * frac,
                pos[idx].y + (pos[next].y - pos[idx].y) * frac};
    }

    // Прогнозує де буде ціль через dt секунд від currentTime.
    // прогноз = поточна_позиція + швидкість * dt
    Coord extrapolate(const Target& target, float currentTime,
                             float dt, float arrayTimeStep) {
        int          n    = target.timeStepsCount;
        int          idx  = static_cast<int>(std::floor(currentTime / arrayTimeStep)) % n;
        int          next = (idx + 1) % n;
        const Coord* pos  = target.positions;
        float        vx   = (pos[next].x - pos[idx].x) / arrayTimeStep;
        float        vy   = (pos[next].y - pos[idx].y) / arrayTimeStep;
        return interpolate(target, currentTime, arrayTimeStep) + Coord{vx, vy} * dt;
    }

    bool needStop(float desiredDir, float currentDir, float angleThreshold) {
        return std::fabs(normalizeAngle(desiredDir - currentDir)) > angleThreshold;
    }

    // Розраховує час польоту дрона з currentPos до targetPos.
    // Враховує: гальмування + поворот (якщо кут великий), розгін, гальмування в кінці.
    float calcTimeOfFlight(
        Coord currentPos, const Coord& targetPos,
        float currentDir, float angleThreshold, float angularSpeed,
        float currentSpeed, float maxSpeed, float accelPath)
    {
        float totalTime  = 0.0f;
        float desiredDir = std::atan2(targetPos.y - currentPos.y, targetPos.x - currentPos.x);
        float a          = calcAccel(maxSpeed, accelPath);

        if (needStop(desiredDir, currentDir, angleThreshold)) {
            float pathToStop = accelPathFromSpeed(currentSpeed, a);
            currentPos.x += std::cos(currentDir) * pathToStop;
            currentPos.y += std::sin(currentDir) * pathToStop;
            totalTime    += accelTimeFromDist(pathToStop, a);
            totalTime    += std::fabs(normalizeAngle(desiredDir - currentDir)) / angularSpeed;
            currentSpeed  = 0.0f;
            currentDir    = desiredDir;
        }

        float dist           = calcDistance(currentPos, targetPos);
        float deceleratePath = std::min(accelPathFromSpeed(maxSpeed, a), dist);
        totalTime += accelTimeFromDist(deceleratePath, a);
        dist      -= deceleratePath;

        float accelDist = std::max(0.0f,
            std::min(accelPathFromSpeed(maxSpeed, a) - accelPathFromSpeed(currentSpeed, a), dist));
        totalTime += accelTimeFromDist(accelDist, a);
        dist      -= accelDist;

        totalTime += dist / maxSpeed;
        return totalTime;
    }

public:
    AnalyticalSolver(const DroneConfig& cfg, const AmmoParams& ammo) {
        ready_ = calcAmmoDropTime(cfg, ammo, ammoFlightTime_);
        if (ready_) {
            hDist_ = calcAmmoHDistance(cfg, ammo, ammoFlightTime_);
            LOG("Ammo flight time: " << ammoFlightTime_ << " s, horizontal distance: " << hDist_ << " m");
        }
    }

    bool solve(
        const Coord&       dronePos,
        float              droneDir,
        float              droneSpeed,
        const Target&      target,
        float              currentTime,
        bool               alreadyApproaching,
        const DroneConfig& cfg,
        const AmmoParams&,
        Coord& outFirePos,
        Coord& outIntermediatePos,
        bool&  outHasIntermediate,
        float& outTotalTime,
        Coord& outPredictedTarget,
        Coord& outAimPoint) override
    {
        if (!ready_) return false;

        // Ітеративне уточнення позиції цілі: повторюємо NUM_APPROX_STEPS разів або до збіжності.
        float totalTime = 0.0f;
        for (int iter = 0; iter < NUM_APPROX_STEPS; ++iter) {
            float prevTotal      = totalTime;
            outPredictedTarget   = extrapolate(target, currentTime,
                                               prevTotal + ammoFlightTime_,
                                               cfg.arrayTimeStep);

            getIntermediateAndDropPoint(dronePos, outPredictedTarget, hDist_, cfg.accelPath,
                outIntermediatePos, outFirePos, outHasIntermediate);

            // Якщо дрон вже летить до цієї цілі — пропускаємо проміжну точку
            if (outHasIntermediate && alreadyApproaching)
                outHasIntermediate = false;

            if (outHasIntermediate) {
                totalTime = calcTimeOfFlight(dronePos, outIntermediatePos,
                    droneDir, cfg.turnThreshold, cfg.angularSpeed,
                    droneSpeed, cfg.attackSpeed, cfg.accelPath);
                float dirToFire = std::atan2(outFirePos.y - outIntermediatePos.y,
                                              outFirePos.x - outIntermediatePos.x);
                totalTime += std::fabs(normalizeAngle(dirToFire - droneDir)) / cfg.angularSpeed;
                totalTime += calcTimeOfFlight(outIntermediatePos, outFirePos,
                    dirToFire, cfg.turnThreshold, cfg.angularSpeed,
                    cfg.attackSpeed, cfg.attackSpeed, cfg.accelPath);
            } else {
                totalTime = calcTimeOfFlight(dronePos, outFirePos,
                    droneDir, cfg.turnThreshold, cfg.angularSpeed,
                    droneSpeed, cfg.attackSpeed, cfg.accelPath);
            }

            if (std::fabs(totalTime - prevTotal) < TIME_CHANGE_THRESHOLD) break;
        }

        outTotalTime = totalTime;
        outAimPoint  = dronePos + Coord{std::cos(droneDir), std::sin(droneDir)} * hDist_;
        return true;
    }
};

// ============================================================
// Factory
// ============================================================

enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

IBallisticSolver* createSolver(SolverType type,
    const DroneConfig& cfg, const AmmoParams& ammo)
{
    switch (type) {
        case SolverType::ANALYTICAL: return new AnalyticalSolver(cfg, ammo);
    }
    return nullptr;
}

ITargetProvider* createProvider(ProviderType type, const char* filename) {
    switch (type) {
        case ProviderType::JSON: return new JsonTargetProvider(filename);
    }
    return nullptr;
}

IConfigLoader* createLoader(LoaderType type,
    const char* configFile, const char* ammoFile)
{
    switch (type) {
        case LoaderType::FILE: return new FileConfigLoader(configFile, ammoFile);
    }
    return nullptr;
}

// ============================================================
// MissionProcessor
// ============================================================

class MissionProcessor {
    ITargetProvider*  targets_;
    IBallisticSolver* solver_;
    IConfigLoader*    loader_;

    DroneConfig cfg_{};
    AmmoParams  ammo_{};

    DroneState droneState_       = STOPPED;
    Coord      currentPos_       = {};
    float      currentDir_       = 0.0f;
    float      currentSpeed_     = 0.0f;
    float      currentTime_      = 0.0f;
    int        currentTargetIdx_ = -1;
    float      acceleration_     = 0.0f;

    SimStep* steps_         = nullptr;
    int      recordedSteps_ = 0;

    bool initialized_ = false;
    bool done_        = false;

public:
    MissionProcessor(ITargetProvider* t, IBallisticSolver* s, IConfigLoader* l)
        : targets_(t), solver_(s), loader_(l) {}

    ~MissionProcessor() { delete[] steps_; }

    bool init() {
        cfg_  = loader_->getConfig();
        ammo_ = loader_->getAmmoParams();

        acceleration_     = calcAccel(cfg_.attackSpeed, cfg_.accelPath);
        currentPos_       = cfg_.initialPos;
        currentDir_       = cfg_.initialDir;
        currentSpeed_     = 0.0f;
        currentTime_      = 0.0f;
        currentTargetIdx_ = -1;
        recordedSteps_    = 0;
        droneState_       = STOPPED;
        done_             = false;

        delete[] steps_;
        steps_ = new SimStep[MAX_STEPS];

        initialized_ = true;
        return true;
    }

    bool hasNext() const {
        return initialized_ && !done_ && recordedSteps_ < MAX_STEPS;
    }

    SimStep step() {
        // STEP 1: вибір найближчої цілі
        // Для кожної цілі solver ітеративно уточнює прогноз позиції та час польоту.
        // Обираємо ціль з найменшим сумарним часом польоту дрона.
        int   bestTarget   = -1;
        float bestTime     = 1e30f;
        Coord bestFirePos  = {};
        Coord bestPredPos  = {};
        Coord bestAimPoint = {};

        for (int i = 0; i < targets_->getTargetCount(); ++i) {
            float totalTime = 0.0f;
            Coord firePos{}, interPos{}, predPos{}, aimPoint{};
            bool  hasInter = false;

            bool alreadyApproaching = (i == currentTargetIdx_)
                && (droneState_ == MOVING || droneState_ == ACCELERATING);

            if (!solver_->solve(currentPos_, currentDir_, currentSpeed_,
                    targets_->getTarget(i), currentTime_, alreadyApproaching,
                    cfg_, ammo_, firePos, interPos, hasInter, totalTime,
                    predPos, aimPoint))
                continue;

            if (totalTime < bestTime) {
                bestTime     = totalTime;
                bestTarget   = i;
                bestFirePos  = firePos;
                bestPredPos  = predPos;
                bestAimPoint = aimPoint;
            }
        }

        if (bestTarget == -1) {
            LOG("Error: no valid target found at time " << currentTime_);
            done_ = true;
            return {};
        }
        currentTargetIdx_ = bestTarget;

        // STEP 2: бажаний напрямок і кут повороту
        float desiredDir = std::atan2(bestFirePos.y - currentPos_.y,
                                       bestFirePos.x - currentPos_.x);
        float deltaAngle = normalizeAngle(desiredDir - currentDir_);
        float deltaPath  = 0.0f;

        // STEP 3: автомат станів дрона
        // Кожен крок симуляції:
        //   - оновлюємо швидкість і стан відповідно до поточного стану
        //   - рахуємо deltaPath — шлях пройдений за цей крок (метри)
        //   - (prevSpeed + currentSpeed) / 2 * dt — усереднена швидкість за крок
        //     дає точніший результат ніж просто speed * dt при прискоренні/гальмуванні
        switch (droneState_) {
            case STOPPED:
                currentSpeed_ = 0.0f;
                if (std::fabs(deltaAngle) > cfg_.turnThreshold)
                    droneState_ = TURNING;
                else { currentDir_ = desiredDir; droneState_ = ACCELERATING; }
                break;

            case TURNING: {
                float da = normalizeAngle(desiredDir - currentDir_);
                if (std::fabs(da) <= cfg_.angularSpeed * cfg_.simTimeStep) {
                    currentDir_ = desiredDir;
                    droneState_ = ACCELERATING;
                } else {
                    currentDir_ += (da > 0 ? 1.0f : -1.0f) * cfg_.angularSpeed * cfg_.simTimeStep;
                    currentDir_  = normalizeAngle(currentDir_);
                }
                break;
            }

            case ACCELERATING: {
                if (std::fabs(deltaAngle) > cfg_.turnThreshold && currentSpeed_ > 0.0f) {
                    droneState_ = DECELERATING;
                    float prev  = currentSpeed_;
                    currentSpeed_ -= acceleration_ * cfg_.simTimeStep;
                    if (currentSpeed_ <= 0.0f) { currentSpeed_ = 0.0f; droneState_ = STOPPED; }
                    deltaPath = (prev + currentSpeed_) / 2.0f * cfg_.simTimeStep;
                } else {
                    if (std::fabs(deltaAngle) <= cfg_.turnThreshold) currentDir_ = desiredDir;
                    float prev = currentSpeed_;
                    currentSpeed_ += acceleration_ * cfg_.simTimeStep;
                    if (currentSpeed_ >= cfg_.attackSpeed) {
                        currentSpeed_ = cfg_.attackSpeed;
                        droneState_   = MOVING;
                    }
                    deltaPath = (prev + currentSpeed_) / 2.0f * cfg_.simTimeStep;
                }
                break;
            }

            case MOVING: {
                if (std::fabs(deltaAngle) > cfg_.turnThreshold) {
                    droneState_ = DECELERATING;
                    float prev  = currentSpeed_;
                    currentSpeed_ -= acceleration_ * cfg_.simTimeStep;
                    if (currentSpeed_ <= 0.0f) { currentSpeed_ = 0.0f; droneState_ = STOPPED; }
                    deltaPath = (prev + currentSpeed_) / 2.0f * cfg_.simTimeStep;
                } else {
                    if (std::fabs(deltaAngle) <= cfg_.turnThreshold) currentDir_ = desiredDir;
                    deltaPath = currentSpeed_ * cfg_.simTimeStep;
                }
                break;
            }

            case DECELERATING: {
                float prev = currentSpeed_;
                currentSpeed_ -= acceleration_ * cfg_.simTimeStep;
                if (currentSpeed_ <= 0.0f) { currentSpeed_ = 0.0f; droneState_ = STOPPED; }
                deltaPath = (prev + currentSpeed_) / 2.0f * cfg_.simTimeStep;
                break;
            }

            default:
                LOG("Error: unknown drone state");
                done_ = true;
                return {};
        }

        // STEP 4: оновлення позиції
        currentPos_.x += std::cos(currentDir_) * deltaPath;
        currentPos_.y += std::sin(currentDir_) * deltaPath;

        // STEP 5: запис кроку і просування часу
        currentTime_ += cfg_.simTimeStep;
        SimStep s = {
            .pos             = currentPos_,
            .direction       = currentDir_,
            .state           = static_cast<int>(droneState_),
            .targetIdx       = currentTargetIdx_,
            .dropPoint       = bestFirePos,
            .aimPoint        = bestAimPoint,
            .predictedTarget = bestPredPos
        };
        steps_[recordedSteps_++] = s;

        DEBUG("Step: "   << recordedSteps_
            << ", Time: "  << currentTime_
            << ", Pos: ("  << currentPos_.x << ", " << currentPos_.y << ")"
            << ", Dir: "   << currentDir_
            << ", State: " << static_cast<int>(droneState_)
            << ", Target: "<< currentTargetIdx_);

        // STEP 6: перевірка умови скидання
        if (droneState_ == MOVING
                && calcDistance(currentPos_, bestFirePos) <= cfg_.hitRadius * 0.2f)
            done_ = true;

        return s;
    }

    void reset() {
        if (!initialized_) return;
        currentPos_       = cfg_.initialPos;
        currentDir_       = cfg_.initialDir;
        currentSpeed_     = 0.0f;
        currentTime_      = 0.0f;
        currentTargetIdx_ = -1;
        recordedSteps_    = 0;
        droneState_       = STOPPED;
        done_             = false;
    }

    void changeSolver(IBallisticSolver* s) { solver_ = s; }

    bool writeOutput(const char* filename) const {
        return writeOutputToFile(filename, steps_, recordedSteps_);
    }

    int  getRecordedSteps() const { return recordedSteps_; }
    bool isDone()           const { return done_; }
};

// ============================================================
// main
// ============================================================

int main() {
    IConfigLoader* loader = createLoader(LoaderType::FILE, "config.json", "ammo.json");
    if (!loader->load()) {
        LOG("Failed to load configuration");
        delete loader;
        return 1;
    }

    DroneConfig cfg = loader->getConfig();
    LOG("Drone and simulation config:");
    LOG("  Initial position: (" << cfg.initialPos.x << ", " << cfg.initialPos.y << ")");
    LOG("  Altitude: "          << cfg.altitude      << " m");
    LOG("  Initial direction: " << cfg.initialDir    << " rad");
    LOG("  Attack speed: "      << cfg.attackSpeed   << " m/s");
    LOG("  Acceleration path: " << cfg.accelPath     << " m");
    LOG("  Ammo type: "         << cfg.ammoName);
    LOG("  Array time step: "   << cfg.arrayTimeStep << " s");
    LOG("  Sim time step: "     << cfg.simTimeStep   << " s");
    LOG("  Hit radius: "        << cfg.hitRadius     << " m");
    LOG("  Angular speed: "     << cfg.angularSpeed  << " rad/s");
    LOG("  Turn threshold: "    << cfg.turnThreshold << " rad");

    ITargetProvider* provider = createProvider(ProviderType::JSON, "targets.json");
    if (provider->getTargetCount() == 0) {
        LOG("Failed to load targets from targets.json");
        delete provider;
        delete loader;
        return 1;
    }

    IBallisticSolver* solver = createSolver(SolverType::ANALYTICAL,
        loader->getConfig(), loader->getAmmoParams());

    MissionProcessor mission(provider, solver, loader);
    if (!mission.init()) {
        LOG("Failed to initialize mission");
        delete solver;
        delete provider;
        delete loader;
        return 1;
    }

    while (mission.hasNext())
        mission.step();

    if (!mission.writeOutput("simulation.json"))
        LOG("Error writing output to file");

    LOG("Simulation completed. Recorded steps: " << mission.getRecordedSteps());
    if (mission.getRecordedSteps() >= MAX_STEPS)
        LOG("Warning: simulation reached MAX_STEPS without drop");

    delete solver;
    delete provider;
    delete loader;
    return 0;
}
