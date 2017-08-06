#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <limits>
#include <unordered_map>


#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  
  /*
   *Init Particle Filter
   * Set the number of particles and initialize all particles to first position (based on
   * estimates of x, y, theta and their uncertainties from GPS) with random Gaussian noise.
   */
  
  //create gaussian distributions for x, y and theta
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  //set the number of particles
  num_particles = 50; 

  //create a temp vector of particles
  vector<Particle> n_particles;
  //create the particles
  for(unsigned int i=0; i < num_particles; i++){    
    Particle p = {};
    p.id         = i;
    //add noise
    p.x          = dist_x(gen);
    p.y          = dist_y(gen);
    p.theta      = dist_theta(gen);
    p.weight     = 1;
    n_particles.push_back(p);    
  }
  particles = n_particles;
  is_initialized = true;
  cout << "INIT OK\n";
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate){

  /*
   * Move the particles (Add measurements to each particle and add random Gaussian noise).
   */
  
  //create normal distributions (Gaussian) for x, y and theta
  default_random_engine gen;
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);
  
  //loop particles and access by reference
  for(Particle& p: particles){
    //compute new position and theta
    double new_theta = 0;
    double new_x = 0;
    double new_y = 0;
    //check if yaw_rate is equal to zero
    if(abs(yaw_rate) == 0){
      //moving straight
      new_x = p.x + (velocity*delta_t*cos(p.theta));
      new_y = p.y + (velocity*delta_t*sin(p.theta));
      new_theta = p.theta;
    } else {
      //turning
      new_theta = p.theta + yaw_rate*delta_t;
      new_x = p.x + (velocity/yaw_rate)*(sin(new_theta) - sin(p.theta));
      new_y = p.y + (velocity/yaw_rate)*(cos(p.theta) - cos(new_theta));      
    }
    //update particle
    p.x = new_x + dist_x(gen);
    p.y = new_y + dist_y(gen);
    p.theta = new_theta + dist_theta(gen); 
  }
  cout << "PREDICT OK\n";
}
  

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  /*
   * Associate for each observation the closest landmark (closest neighbor)
   */

  //loop all observations and access by reference
  for(LandmarkObs& o: observations){
    double min = numeric_limits<double>::max();    
    for(LandmarkObs& p: predicted){
      double d = dist(o.x, o.y, p.x, p.y);
      if(d < min){	
	min = d;
	o.id = p.id;
      }
    }    
  }  
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

  /*
   * Update the weights of each particle following the procedure below:
   * 1 - select the landmarks in the particle range;
   * 2 - transform the measurement observations from the VEHICLE's coordinate to MAP's coordinate;
   * 3 - associat each observation to the closest landmark;
   * 4 - update particle's weight according to the observed distance and the actual position of 
   * each landmark.
   */

  //loop particles and access by reference
  for(Particle& p : particles){    

    //check range, get close landmarks
    vector<LandmarkObs> predictions;    
    for(Map::single_landmark_s& landmark : map_landmarks.landmark_list){
      double d = dist(landmark.x_f, landmark.y_f, p.x, p.y);      
      if(d < sensor_range){	
	LandmarkObs new_obs = {};
	new_obs.id = landmark.id_i;
	new_obs.x  = landmark.x_f;
	new_obs.y  = landmark.y_f;
	predictions.push_back(new_obs);
      }
    }
        
    //transform observations to map's coordinate
    //helper variables
    const double x = p.x;
    const double y = p.y;
    const double theta = p.theta;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);
    //temp vector for transformed observations
    vector<LandmarkObs> particle_observations;    
    for(LandmarkObs& o : observations){
      LandmarkObs no = {};
      no.id = o.id;
      //Udacity Transformation (not working)
      //no.x +=  x*cos_theta + y*sin_theta;
      //no.y += -x*sin_theta + y*cos_theta;
      //http://planning.cs.uiuc.edu/node99.html Transformation
      no.x = o.x*cos_theta - o.y*sin_theta + x;
      no.y = o.x*sin_theta + o.y*cos_theta + y;     
      particle_observations.push_back(no);
    }
    
    //data association between landmarks and observations
    dataAssociation(predictions, particle_observations);   
    
    //record associations
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    //compute weights
    double weight=1;
    double std_2_pi = 2.0*M_PI*std_landmark[0]*std_landmark[1];
    double std_x_2  = 2.0*std_landmark[0]*std_landmark[0];
    double std_y_2  = 2.0*std_landmark[1]*std_landmark[1];
    for(LandmarkObs& o : particle_observations){
      //recover associated landmark
      Map::single_landmark_s m =  map_landmarks.landmark_list.at(o.id -1);//map_id->second];
      //compute Multivariate-Gaussian probability
      double e1 = pow(o.x - m.x_f, 2);
      double e2 = pow(o.y - m.y_f, 2);
      double e = (e1/std_x_2 + e2/std_y_2);
      double ee = exp(-e);
      double w = ee/std_2_pi;
      //prod of all weights
      weight *= w;
      //record association
      associations.push_back(o.id);
      sense_x.push_back(o.x);
      sense_y.push_back(o.y);      
    }
    //update particle with final weight
    p.weight = weight;
    //insert into weight vector
    weights.push_back(weight);
    //update particle's associations
    SetAssociations(p, associations, sense_x, sense_y);    
  }
  cout << "UPDATE WEIGHTS OK\n";
}

void ParticleFilter::resample() {

  /*
  * Resample particles with replacement with probability proportional to their weight. 
  */

  //temp vectors for the new set of resampled particles
  vector<Particle> n_particles;

  //create discrete distibution from particles weights
  random_device rd;  
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());
  
  //resample  
  for(unsigned int i=0; i < num_particles; i++){      
    const int index = d(gen);  
    const Particle new_p = particles[index];
    n_particles.push_back(new_p);
  }
  //clear Particle vectors
  particles.clear();
  weights.clear();
  //new set of particles
  particles = n_particles;
  cout << "RESAMPLE OK\n";
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

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
