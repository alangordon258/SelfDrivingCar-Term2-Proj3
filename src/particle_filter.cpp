/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    
    
    default_random_engine gen;
    int i;
    num_particles=100;
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    for (i=0;i<num_particles;i++) {
        Particle particle;
        particle.id=i;
        particle.x=dist_x(gen);
        particle.y=dist_y(gen);
        particle.theta=dist_theta(gen);
        particle.weight=1.0;
        particles.push_back(particle);
    }
    
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    
    // define normal distributions for sensor noise

    double xzero, yzero, thetazero, thetafinal;
    Particle particle;
    vector<Particle>::iterator it;
    default_random_engine gen;
    //    normal_distribution<double> dist_velocity(velocity, std_pos[0]);
    //    normal_distribution<double> dist_yaw_rate(yaw_rate, std_pos[2]);
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);
    int i;
    for(i = 0; i < num_particles; i++)    {
        xzero=particles[i].x;
        yzero=particles[i].y;
        thetazero=particles[i].theta;
        thetafinal=thetazero+yaw_rate*delta_t;
        if (fabs(yaw_rate) < 0.00001) {
             particles[i].x = xzero + velocity * delta_t * cos(particles[i].theta);
             particles[i].y = yzero + velocity * delta_t * sin(particles[i].theta);
         }
        else {
            particles[i].x=xzero+(velocity/yaw_rate)*(sin(thetafinal)-sin(thetazero));
            particles[i].y=yzero+(velocity/yaw_rate)*(cos(thetazero)-cos(thetafinal));
            particles[i].theta=thetafinal;
        }
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.
    vector<LandmarkObs>::iterator it_pred, it_obs;
    double distance, min_distance;
    
    for (unsigned int i = 0; i < observations.size(); i++)   {
        min_distance=numeric_limits<double>::max();
        for (unsigned int j = 0; j < predicted.size(); j++) {
            distance=dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
            if (distance < min_distance) {
                observations[i].id=predicted[j].id;
                min_distance=distance;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> &observations,const Map &map_landmarks) {
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
    //   for the fact that the map's y-axis actually points downwards.)
    //   http://planning.cs.uiuc.edu/node99.html
    vector<LandmarkObs>::iterator it_obs, it_predictions;
    LandmarkObs observation, prediction;
    double xm, ym, xc, yc;
    double xp, yp, thetap, distance;
    int i;
    for (i = 0; i < num_particles; i++) {
        
        // get the x and y coordinates of the particle
        xp = particles[i].x;
        yp = particles[i].y;
        thetap = particles[i].theta;
        
        // find the map landmark that are within sensor range of the particle
        vector<LandmarkObs> transformed_obs, predictions;
        Map::single_landmark_s landmark;
        // add each map landmark to the vector of predictions
        std::vector<Map::single_landmark_s> landmark_list=map_landmarks.landmark_list;
        std::vector<Map::single_landmark_s>::iterator it_landmark;
        for (it_landmark=landmark_list.begin(); it_landmark != landmark_list.end(); it_landmark++) {
            landmark=*it_landmark;
            distance=dist(xp,yp,(double)landmark.x_f,(double)landmark.y_f);
            if (distance < sensor_range) {
                LandmarkObs landmarkobj={landmark.id_i,landmark.x_f,landmark.y_f};
                predictions.push_back(landmarkobj);
            }
        }
        
        // create a vector of the observations transformed from vehicle coordinates to map coordinates
        
        for(it_obs = observations.begin(); it_obs != observations.end(); it_obs++)  {
            observation=*it_obs;
            xc=observation.x;
            yc=observation.y;
            xm=xp+cos(thetap)*xc-sin(thetap)*yc;
            ym=yp+sin(thetap)*xc+cos(thetap)*yc;
            LandmarkObs transformed_landmark={ observation.id, xm, ym};
            transformed_obs.push_back(transformed_landmark);
        }
        
        // associate the transformed observations with the closest landmark
        dataAssociation(predictions, transformed_obs);
        
        // initialize weights
        particles[i].weight = 1.0;
        
        for(it_obs = transformed_obs.begin(); it_obs != transformed_obs.end(); it_obs++)  {
            double x, y, mu_x, mu_y, x_dist, y_dist, exp_raise_val, p;
            int id_of_closest;
            observation=*it_obs;
            x=observation.x;
            y=observation.y;
            id_of_closest=observation.id;
            for(it_predictions = predictions.begin(); it_predictions != predictions.end(); it_predictions++)  {
                prediction=*it_predictions;
                if (prediction.id==observation.id) {
                    mu_x=prediction.x;
                    mu_y=prediction.y;
                }
            }
            x_dist=x-mu_x;
            y_dist=y-mu_y;
            exp_raise_val=x_dist*x_dist/(2*std_landmark[0]*std_landmark[0])+y_dist*y_dist/(2*std_landmark[1]*std_landmark[1]);
            p=1/(2*M_PI*std_landmark[0]*std_landmark[1])*exp(-exp_raise_val);
            particles[i].weight*=p;
        }
    }
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    default_random_engine gen;
    int i;
    vector<Particle> resampled_particles;
    
    // create a vector of the current weights
    vector<double> weights;
    for (i = 0; i < num_particles; i++)
        weights.push_back(particles[i].weight);
    
    // generate random starting index for resampling wheel
    uniform_int_distribution<int> int_dist(0, num_particles-1);
    int index = int_dist(gen);
    
    // get max weight
    double mw = *max_element(weights.begin(), weights.end());
    
    // create a uniform random distribution
    uniform_real_distribution<double> real_dist(0.0, mw);
    
    double beta = 0.0;
    
    // Implement Sebastian's resample wheel algorithm in C++
    for (i = 0; i < num_particles; i++) {
        beta += real_dist(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampled_particles.push_back(particles[index]);
    }
    
    particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                         const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    
    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
    vector<double> v = best.sense_x;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best)
{
    vector<double> v = best.sense_y;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
/*
void ParticleFilter::write(std::string filename) {
    // You don't need to modify this file.
    std::ofstream dataFile;
    dataFile.open(filename, std::ios::app);
    for (int i = 0; i < num_particles; ++i) {
        dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
    }
    dataFile.close();
}
 */
