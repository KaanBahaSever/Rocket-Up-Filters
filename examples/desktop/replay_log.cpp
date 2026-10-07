// Run the flight computer on your own recorded sensor log (real flights, ground tests).
//
//   replay_log flight.csv                          (log of examples/BMP280_MPU6050)
//   replay_log log.csv --time time_s --time-scale 1 --pressure press_hPa --pressure-scale 100 \
//                      --accel acc_z_g --accel-scale 9.80665 --main 500
//
// Columns can be given by header name or 0-based index. Lines starting with '#' are ignored,
// the delimiter (',' ';' or tab) is detected automatically. A pressure equal to the previous
// one is treated as "no new barometer sample" (multi-rate logs).
//
// Options
//   --time COL --time-scale S          time column and factor to seconds   (t_ms, 0.001)
//   --pressure COL --pressure-scale S  pressure column and factor to Pa    (pressure_Pa, 1)
//   --accel COL --accel-scale S        axial acceleration and factor to m/s^2 (axial, 1)
//   --main M                           main deployment altitude, m          (600)
//   --launch-g G                       launch threshold in g                (3)
//   --baro-noise M --accel-noise A     filter noise settings                (0.4, 0.6)
//   --calibration N                    ground pressure samples              (100)
//   --out FILE                         estimates CSV (default <log>_filtered.csv)

#include <RocketUpFilters.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<std::string> split(const std::string& line, char sep) {
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false;
    for (char c : line) {
        if (c == '"') quoted = !quoted;
        else if (c == sep && !quoted) {
            out.push_back(cur);
            cur.clear();
        } else if (c != '\r') cur.push_back(c);
    }
    out.push_back(cur);
    for (auto& s : out) {
        while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
        while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.pop_back();
    }
    return out;
}

bool isNumber(const std::string& s) {
    char* end = nullptr;
    std::strtod(s.c_str(), &end);
    return !s.empty() && end && *end == '\0';
}

int resolve(const std::string& col, const std::vector<std::string>& header) {
    for (size_t i = 0; i < header.size(); ++i)
        if (header[i] == col) return static_cast<int>(i);
    if (isNumber(col)) return std::atoi(col.c_str());
    return -1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: replay_log <log.csv> [--time COL] [--pressure COL] [--accel COL] [... see source]\n";
        return 1;
    }
    std::map<std::string, std::string> opt = {{"time", "t_ms"},        {"time-scale", "0.001"}, {"pressure", "pressure_Pa"},
                                              {"pressure-scale", "1"}, {"accel", "axial"},      {"accel-scale", "1"},
                                              {"main", "600"},         {"launch-g", "3"},       {"baro-noise", "0.4"},
                                              {"accel-noise", "0.6"},  {"calibration", "100"}};
    for (int i = 2; i + 1 < argc; i += 2) {
        std::string k = argv[i];
        if (k.rfind("--", 0) != 0) {
            std::cerr << "unexpected argument " << k << "\n";
            return 1;
        }
        opt[k.substr(2)] = argv[i + 1];
    }
    const std::string path = argv[1];
    const std::string outPath = opt.count("out") ? opt["out"] : path + "_filtered.csv";
    std::ifstream in(path);
    if (!in) {
        std::cerr << "cannot open " << path << "\n";
        return 1;
    }

    rufilters::FlightComputer<double>::Config cfg;
    cfg.detector.mainAltitude = std::atof(opt["main"].c_str());
    cfg.detector.launchAccel = std::atof(opt["launch-g"].c_str()) * 9.80665;
    cfg.altitudeNoise = std::atof(opt["baro-noise"].c_str());
    cfg.accelNoise = std::atof(opt["accel-noise"].c_str());
    cfg.calibrationSamples = std::atoi(opt["calibration"].c_str());
    rufilters::FlightComputer<double> fc(cfg);
    const double ts = std::atof(opt["time-scale"].c_str()), ps = std::atof(opt["pressure-scale"].c_str()),
                 as = std::atof(opt["accel-scale"].c_str());

    std::string line;
    std::vector<std::string> header;
    char sep = ',';
    int cT = -1, cP = -1, cA = -1;
    double lastP = -1.0, maxBaroAlt = -1e9;
    long rows = 0;
    std::ostringstream out;
    out << "time_s,pressure_Pa,baro_altitude_m,altitude_m,velocity_mps,acceleration_mps2,phase\n";
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (cT < 0) {
            sep = line.find(';') != std::string::npos ? ';' : (line.find('\t') != std::string::npos ? '\t' : ',');
            const auto f = split(line, sep);
            if (!isNumber(f[0])) header = f;
            cT = resolve(opt["time"], header);
            cP = resolve(opt["pressure"], header);
            cA = resolve(opt["accel"], header);
            if (cT < 0 || cP < 0 || cA < 0) {
                std::cerr << "column not found (time '" << opt["time"] << "', pressure '" << opt["pressure"]
                          << "', accel '" << opt["accel"] << "'). Header:";
                for (auto& h : header) std::cerr << " [" << h << "]";
                std::cerr << "\n";
                return 1;
            }
            if (!header.empty()) continue;
        }
        const auto f = split(line, sep);
        const int need = std::max(cT, std::max(cP, cA));
        if (static_cast<int>(f.size()) <= need || !isNumber(f[cT]) || !isNumber(f[cP]) || !isNumber(f[cA])) continue;
        const double t = std::atof(f[cT].c_str()) * ts;
        const double p = std::atof(f[cP].c_str()) * ps;
        const double a = std::atof(f[cA].c_str()) * as;
        const bool fresh = p != lastP;
        lastP = p;
        ++rows;
        const uint8_t ev = fc.update(t, p, a, fresh);
        const double baroAlt = rufilters::pressureToAltitude<double>(p, fc.groundPressure());
        if (fc.phase() != rufilters::Phase::Calibrating && baroAlt > maxBaroAlt) maxBaroAlt = baroAlt;
        if (ev & rufilters::EventLaunch) std::printf("t = %8.3f s  launch\n", t);
        if (ev & rufilters::EventBurnout) std::printf("t = %8.3f s  burnout (v = %.1f m/s, h = %.1f m)\n", t, fc.velocity(), fc.altitude());
        if (ev & rufilters::EventApogee)
            std::printf("t = %8.3f s  apogee: filtered %.1f m (raw barometer max %.1f m)\n", t,
                        fc.detector().apogeeAltitude(), maxBaroAlt);
        if (ev & rufilters::EventMain) std::printf("t = %8.3f s  main deployment altitude (h = %.1f m)\n", t, fc.altitude());
        if (ev & rufilters::EventLanded) std::printf("t = %8.3f s  landed\n", t);
        out << t << "," << p << "," << baroAlt << "," << fc.altitude() << "," << fc.velocity() << ","
            << fc.acceleration() << "," << rufilters::phaseName(fc.phase()) << "\n";
    }
    std::ofstream(outPath) << out.str();
    std::printf("%ld samples, ground pressure %.1f Pa, final phase: %s\nestimates written to %s\n", rows,
                fc.groundPressure(), rufilters::phaseName(fc.phase()), outPath.c_str());
    return 0;
}
