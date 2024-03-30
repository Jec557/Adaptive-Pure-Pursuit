# Adaptive Pure Pursuit Controller used by VEX Robotics Team 955R 
## Created by Joshua Chong
Using this program will require the the use of an external operating system to run the calculations, as the VEX brain cannot handle it for some reason. The operating system we used was [[cpp.sh](cpp.sh)](https://cpp.sh/).

Paste the following into the operating system and add your waypoints where it says to under main. Paste the output of this program where it says to at the bottom of odometry in the Pure Pursuit program. You can also paste the calculated path into a graphing calculator such as Desmos to view your path. Just make sure to change the brackets to parentesis in the code that displays the path so that it easy to copy:

```
#include <iostream>
#include <vector>
#include <cmath>

// Function to calculate distance between two points
double distance(const std::vector<double>& a, const std::vector<double>& b) {
    return sqrt(pow((b[0] - a[0]), 2) + pow((b[1] - a[1]), 2));
}

// Function to inject waypoints at a regular interval between given waypoints
std::vector<std::vector<double>> injectWaypoints(const std::vector<std::vector<double>>& waypoints, double interval) {
    std::vector<std::vector<double>> injectedPoints;

    for (int i = 0; i < waypoints.size() - 1; ++i) {
        const std::vector<double>& startPoint = waypoints[i];
        const std::vector<double>& endPoint = waypoints[i + 1];

        double totalDistance = distance(startPoint, endPoint);
        int numIntervals = static_cast<int>(totalDistance / interval);

        for (int j = 0; j <= numIntervals; ++j) {
            double ratio = static_cast<double>(j + 1) / (numIntervals + 1);
            double newX = startPoint[0] + ratio * (endPoint[0] - startPoint[0]);
            double newY = startPoint[1] + ratio * (endPoint[1] - startPoint[1]);
            injectedPoints.push_back({newX, newY});
        }
    }

    return injectedPoints;
}

// Function to smooth points into a curve
std::vector<std::vector<double>> smoother(const std::vector<std::vector<double>>& path, double a, double b, double tolerance) {
    std::vector<std::vector<double>> newPath = path;
    double change = tolerance;

    while (change >= tolerance) {
        change = 0.0;
        for (size_t i = 1; i < path.size() - 1; ++i) {
            for (size_t j = 0; j < path[i].size(); ++j) {
                double aux = newPath[i][j];
                newPath[i][j] += a * (path[i][j] - newPath[i][j]) + b *
                    (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                change += std::abs(aux - newPath[i][j]);
            }
        }
    }

    return newPath;
}

// Function to calculate curvature of each point
std::vector<double> curve(const std::vector<std::vector<double>>& curvedpath){
    std::vector<double> curvaturepoints;
    curvaturepoints.insert(curvaturepoints.begin(), 0);
    for (int i = 1; i < curvedpath.size() - 1; ++i) {
        double x1 = curvedpath[i][0] + 0.001;
        double y1 = curvedpath[i][1];
        double x2 = curvedpath[i-1][0];
        double y2 = curvedpath[i-1][1];
        double x3 = curvedpath[i+1][0];
        double y3 = curvedpath[i+1][1];
        double k1 = 0.5 * (pow(x1, 2)+pow(y1, 2)-pow(x2, 2)-pow(y2, 2))/(x1-x2);
        double k2 = (y1-y2)/(x1-x2);
        double b = 0.5 * (pow(x2, 2) - 2*x2*k1 + pow(y2, 2) - pow(x3, 2) + 2*x3*k1 - pow(y3, 2))/(x3*k2 - y3 + y2 - x2*k2);
        double a = k1 - k2*b;
        double r = sqrt(pow((x1 - a), 2) + pow((y1 - b), 2));
        double curvature = 1/r;
        if (isnan(curvature) == true) {
            curvaturepoints.insert(curvaturepoints.end(), 0);
        } else {
            curvaturepoints.insert(curvaturepoints.end(), curvature);
        }
    }
    curvaturepoints.insert(curvaturepoints.end(), 0);
    return curvaturepoints;
}


std::vector<double> maxvelocities(const std::vector<double>& curvaturepts, double pathmaxvelocity){
    std::vector<double> maxvpoints;
    for (int i = 0; i < curvaturepts.size(); ++i) {
        maxvpoints.insert(maxvpoints.end(), std::min(pathmaxvelocity, 3/curvaturepts[i])); 
    }
    return maxvpoints;
}

// Function to calculate target velocities of each point
std::vector<double> targetvelocities(const std::vector<std::vector<double>>& smoothedpts, std::vector<double>& oldtargetv, double pathmaxacceleration){
    std::vector<double> newtargetv;
    newtargetv.insert(newtargetv.end(), 0);
    for (int i = smoothedpts.size() - 2; i >= 0; --i) {
        double dist = distance(smoothedpts[i+1], smoothedpts[i]);
        double vf = sqrt(pow(newtargetv[0], 2) + 2*pathmaxacceleration*dist);
        double newv = std::min(oldtargetv[i], vf);
        newtargetv.insert(newtargetv.begin(), newv); 
    }
    return newtargetv;
}

std::vector<std::vector<double>> smoothedPath;
std::vector<double> newvelocities;

double previoustarget = 0;


int main() {
    // Given waypoints (list of points)
    std::vector<std::vector<double>> waypoints = {
    // Add points defining the waypoints here 
    // Example:
      {0, 0},
      {30, 40},
      {60, 15},
      {100, 60},
    
    };

// Set Variables Here
    double weight_on_points = 0.1; // Increase this to make path follow set points more closely
    double weight_on_smoothness = 0.9; // Increase this to increase the smoothness of the path
    double tolerance = 0.001;
    double interval = 6.0; // Interval between injected waypoints
    double max_velocity = 600;
    double max_acceleration = 10;

    // Inject waypoints at a regular interval between the given waypoints
    std::vector<std::vector<double>> injectedWaypoints = injectWaypoints(waypoints, interval);

    //injectedWaypoints.insert(injectedWaypoints.begin(), {0.0, 0.0});

    std::vector<std::vector<double>> smoothedPath = smoother(injectedWaypoints, weight_on_points, weight_on_smoothness, tolerance);

    // Display the smoothed path
    std::cout << "Smoothed Path:\n";
    for (const auto& row : smoothedPath) {
        std::cout << "{" << row[0] << ", " << row[1] << "},\n";
    }
    
    std::vector<double> curvaturelist = curve(smoothedPath);
    
    std::cout << "Curvature of Points\n";
    
    std::vector<double> maxv = maxvelocities(curvaturelist, max_velocity);
    
    std::vector<double> newvelocities = targetvelocities(smoothedPath, maxv, max_acceleration);

    // Display the Velocities of the Path
    std::cout << "New Velocities\n";
    for (int i = 0; i < newvelocities.size(); ++i){
        std::cout << newvelocities[i] << ", ";
    }
}
```
