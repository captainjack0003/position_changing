#include <iostream>
#include <cmath>
#include <vector>
#include <unistd.h>

const double acc = 1;
const double vel = 1;

std::vector<double> desired_joint_acc, desired_joint_vel;
std::vector<double> acc_time, cruise_time, deacc_time;
std::vector<double> ini_pos{0.0, 0.0, 0.0};
std::vector<double> final_pos{3.0, 3.0, 3.0};
int num_joints;

void traj_path_point_to_point()
{
    num_joints = ini_pos.size();
    desired_joint_acc.resize(num_joints);
    desired_joint_vel.resize(num_joints);
    acc_time.resize(num_joints);
    cruise_time.resize(num_joints);
    deacc_time.resize(num_joints);

    for (unsigned int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {
        if (fabs((final_pos[jnt_ctr] - ini_pos[jnt_ctr])) > 1e-5)
        {
            desired_joint_acc[jnt_ctr] = (final_pos[jnt_ctr] - ini_pos[jnt_ctr]) / fabs((final_pos[jnt_ctr] - ini_pos[jnt_ctr])) * acc;
            desired_joint_vel[jnt_ctr] = (final_pos[jnt_ctr] - ini_pos[jnt_ctr]) / fabs((final_pos[jnt_ctr] - ini_pos[jnt_ctr])) * vel;
        }
        else
        {
            desired_joint_acc[jnt_ctr] = 0;
            desired_joint_vel[jnt_ctr] = 0;
        }
    }

    for (unsigned int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {
        double min_pos_travel = 0;

        min_pos_travel = desired_joint_vel[jnt_ctr] * desired_joint_vel[jnt_ctr] / desired_joint_acc[jnt_ctr];

        if (fabs(final_pos[jnt_ctr] - ini_pos[jnt_ctr]) < fabs(min_pos_travel))
        {
            acc_time[jnt_ctr] = sqrt((final_pos[jnt_ctr] - ini_pos[jnt_ctr]) / desired_joint_acc[jnt_ctr]);
            cruise_time[jnt_ctr] = 0;
            deacc_time[jnt_ctr] = acc_time[jnt_ctr];
        }
        else
        {
            acc_time[jnt_ctr] = desired_joint_vel[jnt_ctr] / desired_joint_acc[jnt_ctr];
            cruise_time[jnt_ctr] = (final_pos[jnt_ctr] - ini_pos[jnt_ctr] - desired_joint_acc[jnt_ctr] * pow(acc_time[jnt_ctr], 2)) / desired_joint_vel[jnt_ctr];
            deacc_time[jnt_ctr] = acc_time[jnt_ctr];
        }
        std::cout<<"jnt_ctr : "<<jnt_ctr<<", acc_time: "<<acc_time[jnt_ctr]<<", cruise_time : "<<cruise_time[jnt_ctr]<<", deacc_time : "<<deacc_time[jnt_ctr]<<std::endl;
    }
}

std::vector<double> calculate_pos(double time)
{

    std::vector<double> current_pos;
    current_pos.resize(num_joints);

    

    for (unsigned int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {        
        if (time < acc_time[jnt_ctr])
        {
            std::cout<<"under acc phase \n";
            current_pos[jnt_ctr] = 0.5 * desired_joint_acc[jnt_ctr] * time * time + ini_pos[jnt_ctr];
        }

        else if (time < (acc_time[jnt_ctr] + cruise_time[jnt_ctr]))
        {
            std::cout<<"under cruise phase \n";
            current_pos[jnt_ctr] = ini_pos[jnt_ctr] + 0.5 * desired_joint_acc[jnt_ctr] * pow(acc_time[jnt_ctr], 2) + desired_joint_acc[jnt_ctr] * acc_time[jnt_ctr] * (time - acc_time[jnt_ctr]);
        }

        else if ( time < (deacc_time[jnt_ctr] + cruise_time[jnt_ctr] + acc_time[jnt_ctr]) )
        {
            std::cout<<"under deacc phase \n";
            current_pos[jnt_ctr] = final_pos[jnt_ctr] - 0.5 *desired_joint_acc[jnt_ctr]*(deacc_time[jnt_ctr] + cruise_time[jnt_ctr] + acc_time[jnt_ctr] - time)*(deacc_time[jnt_ctr] + cruise_time[jnt_ctr] + acc_time[jnt_ctr] - time);
        }
    }

    return current_pos;
}

int main()
{
    traj_path_point_to_point();

    FILE *fp;
    fp = fopen("data.csv", "a");
    if (fp == NULL)
    {
        perror("Error opening file");
        return 1;
    }

    for (double t = 0.0; t <= acc_time[0] + cruise_time[0] + deacc_time[0]; t += 0.01)
    {
        std::vector<double> position = calculate_pos(t);
        std::cout << "Joint position at time " << t << "\n"
                  << "position 1 : " << position[0] << "\n"
                  << "position 2 : " << position[1] << "\n"
                  << "position 3 : " << position[2] << std::endl;
        if (fprintf(fp, "%f,%f,%f,%f\n", t, position[0], position[1], position[2]) < 0)
        {
            perror("Error writing to file");
            return 1;
        }
        usleep(1000);
    }
}