// Replay a Rocket-Up flight through the flight computer.
//
//   rocketup run orbit_chaser.project.json -o out        (in the Rocket-Up repository)
//   replay_rocketup out/flight_orbit_chaser.csv [baroNoisePa] [accelNoise]
//
// Reads the true air pressure and axial acceleration from the Rocket-Up CSV, adds sensor noise,
// runs rufilters::FlightComputer at the CSV rate and compares the detected events with the
// true ones. Writes <input>_filtered.csv with true and estimated altitude/velocity.

#include <RocketUpFilters.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false;
    for (char c : line) {
        if (c == '"') quoted = !quoted;
        else if (c == ',' && !quoted) {
            out.push_back(cur);
            cur.clear();
        } else if (c != '\r') cur.push_back(c);
    }
    out.push_back(cur);
    return out;
}

int column(const std::vector<std::string>& header, const std::string& prefix) {
    for (size_t i = 0; i < header.size(); ++i)
        if (header[i].compare(0, prefix.size(), prefix) == 0) return static_cast<int>(i);
    return -1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: replay_rocketup <flight.csv> [baroNoisePa=3] [accelNoise=0.3]\n";
        return 1;
    }
    const double baroNoise = argc > 2 ? std::atof(argv[2]) : 3.0;
    const double accelNoise = argc > 3 ? std::atof(argv[3]) : 0.3;
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 1;
    }
    std::string line;
    std::vector<std::string> header;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        header = splitCsv(line);
        break;
    }
    const int cT = column(header, "Time"), cH = column(header, "Altitude AGL"), cV = column(header, "Vertical velocity"),
              cP = column(header, "Air pressure"), cA = column(header, "Axial acceleration"), cPh = column(header, "Flight phase");
    if (cT < 0 || cH < 0 || cP < 0 || cA < 0 || cV < 0) {
        std::cerr << "not a Rocket-Up flight CSV (need time, altitude, vertical velocity, air pressure, axial acceleration)\n";
        return 1;
    }

    std::mt19937 rng(42);
    std::normal_distribution<double> n(0.0, 1.0);
    rufilters::FlightComputer<float> fc;
    double trueApogee = -1e9, trueApogeeTime = 0, lastT = -1;
    std::ostringstream out;
    out << "time_s,true_altitude_m,est_altitude_m,true_velocity_mps,est_velocity_mps,phase\n";
    // Ground calibration: feed the first sample several times as "pad" data.
    bool primed = false;
    while (std::getline(in, line)) {
        const auto f = splitCsv(line);
        if (static_cast<int>(f.size()) <= cA) continue;
        const double t = std::atof(f[cT].c_str()), h = std::atof(f[cH].c_str()), v = std::atof(f[cV].c_str());
        const double p = std::atof(f[cP].c_str()), a = std::atof(f[cA].c_str());
        if (!primed) {
            for (int i = 0; i < 60; ++i)
                fc.update(static_cast<float>(t - 0.6 + i * 0.01), static_cast<float>(p + baroNoise * n(rng)),
                          static_cast<float>(a + accelNoise * n(rng)));
            primed = true;
        }
        if (t - lastT < 0.0099) continue;  // ~100 Hz like a real sensor loop
        lastT = t;
        const bool descent = cPh >= 0 && std::atof(f[cPh].c_str()) >= 4.0;
        const double axial = descent ? 9.81 + 2.0 * n(rng) : a + accelNoise * n(rng);
        const uint8_t ev = fc.update(static_cast<float>(t), static_cast<float>(p + baroNoise * n(rng)),
                                     static_cast<float>(axial));
        if (h > trueApogee) {
            trueApogee = h;
            trueApogeeTime = t;
        }
        if (ev & rufilters::EventLaunch) std::printf("t = %7.2f s  launch detected\n", t);
        if (ev & rufilters::EventBurnout) std::printf("t = %7.2f s  burnout detected\n", t);
        if (ev & rufilters::EventApogee)
            std::printf("t = %7.2f s  apogee detected: %.1f m (true %.1f m at %.2f s)\n", t,
                        fc.detector().apogeeAltitude(), trueApogee, trueApogeeTime);
        if (ev & rufilters::EventMain) std::printf("t = %7.2f s  main deployment altitude, true altitude %.1f m\n", t, h);
        if (ev & rufilters::EventLanded) std::printf("t = %7.2f s  landing detected\n", t);
        out << t << "," << h << "," << fc.altitude() << "," << v << "," << fc.velocity() << ","
            << rufilters::phaseName(fc.phase()) << "\n";
    }
    const std::string outPath = std::string(argv[1]) + "_filtered.csv";
    std::ofstream(outPath) << out.str();
    std::cout << "estimates written to " << outPath << "\n";
    return 0;
}
