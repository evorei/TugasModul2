#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

const double PI = 3.14159265358979323846;

class RobotArm {
private:
    double link1, link2;
    double theta1, theta2;

public:
    RobotArm(double l1, double l2) : link1(l1), link2(l2), theta1(0), theta2(0) {}

    bool inverseKinematics(double x, double y) {
        double d = (x*x + y*y - link1*link1 - link2*link2) / (2 * link1 * link2);
        if (d < -1 || d > 1) {
            cout << "Target di luar jangkauan." << endl;
            return false;
        }

        theta2 = atan2(sqrt(1 - d*d), d);
        theta1 = atan2(y, x) - atan2(link2 * sin(theta2), link1 + link2 * cos(theta2));

        return true;
    }

    void displayAngles() const {
        cout << "Theta1: " << theta1 * 180 / PI << " degrees" << endl;
        cout << "Theta2: " << theta2 * 180 / PI << " degrees" << endl;
    }

    void saveToFile(const string& filename) const {
        ofstream file;
        file.open(filename);
        if (file.is_open()) {
            file << "Theta1: " << theta1 * 180 / PI << " degrees" << endl;
            file << "Theta2: " << theta2 * 180 / PI << " degrees" << endl;
            file.close();
            cout << "Data saved to " << filename << endl;
        } else {
            cout << "Unable to open file!" << endl;
        }
    }
};

int main() {
    double l1, l2, x, y;
    
    cout << "Masukkan panjang link 1: ";
    cin >> l1;
    cout << "Masukkan panjang link 2: ";
    cin >> l2;
    
    RobotArm robot(l1, l2);

    cout << "Masukkan koordinat target (x, y): ";
    cin >> x >> y;

    if (robot.inverseKinematics(x, y)) {
        robot.displayAngles();
        robot.saveToFile("robot_arm_angles.txt");
    }

    return 0;
}
