#ifndef WHOONLY_INCLUDE
#define WHOONLY_INCLUDE

#include <stdlib.h>
#include <iostream>
#include <fstream>

#include "ffld/timingtools.h"
timeval Start, Stop;

#include "ffld/Intersector.h"

#include "ffld/DPMDetection.h"

using namespace FFLD;
using namespace std;

DPMDetection::DPMDetection ( bool verbose, double overlap, double overlap_all, int padding, int interval )
{
  init ( verbose, overlap, overlap_all, padding, interval );
}

DPMDetection::DPMDetection ( const std::string & modelfile, double threshold, bool verbose, double overlap, double overlap_all, int padding, int interval ) 
{
  init ( verbose, overlap, overlap_all, padding, interval );
  addModel ( "single", modelfile, threshold );
}

void DPMDetection::init ( bool verbose, double overlap, double overlap_all, int padding, int interval )
{
  this->overlap = overlap;
  this->overlap_all = overlap_all;
  this->padding = padding;
  this->interval = interval;
  this->verbose = verbose;
  this->initw = -1;
  this->inith = -1;
}


int DPMDetection::addModel ( const std::string & classname, const std::string & modelfile, double threshold )
{
  // Try to open the mixture
	ifstream in(modelfile.c_str(), ios::binary);
	
	if (!in.is_open()) {
		cerr << "Invalid model file " << modelfile << endl;
    return -1;
	}
	
	Mixture *mixture = new Mixture();
	in >> (*mixture);

  if (mixture->empty()) {
		cerr << "Invalid model file " << modelfile << endl;
    return -1; 
	}

  mixtures.insert( pair<std::string, Mixture *>( classname, mixture ) );
  thresholds.insert ( pair<std::string, double> ( classname, threshold ) );

  return 0;
}

DPMDetection::~DPMDetection()
{
  for ( map<std::string, Mixture *>::iterator i = mixtures.begin(); i != mixtures.end(); i++ )
  {
    Mixture *mixture = i->second;
    if ( mixture != NULL ) 
      delete mixture;
  }
  mixtures.clear();
}

int DPMDetection::detect ( const JPEGImage & image, vector<Detection> & detections )
{
  if ( mixtures.size() == 0 )
    return -1;

  // Compute the HOG features
  if (this->verbose)
  	start();

	HOGPyramid pyramid(image, this->padding, this->padding, this->interval);

  if (pyramid.empty()) {
		cerr << "\nInvalid image!" << endl;
		return -1;
	}

  if (this->verbose) {
    cerr << "Computed HOG features in " << stop() << " ms for an image of size " <<
      image.width() << " x " << image.height() << endl;
    start();
  }

  // Do this only when necessary
  //
  int w = (pyramid.levels()[0].rows() - this->padding + 15) & ~15;
  int h = (pyramid.levels()[0].cols() - this->padding + 15) & ~15;
  if ( w != initw || h != inith )
  {
    if (verbose)
      cerr << "Init values for Patchwork: " << w << " x " << h << endl;

    if (!Patchwork::Init(w,h)) {
      cerr << "\nCould not initialize the Patchwork class" << endl;
      return -1;
    }
    // Initialize the Patchwork class
    if (this->verbose) {
      cerr << "Initialized FFTW in " << stop() << " ms" << endl;
      start();
    }
    
    for ( map<std::string, Mixture *>::iterator i = this->mixtures.begin(); i != this->mixtures.end(); i++ )
    {
      Mixture *mixture = i->second;
      mixture->cacheFilters();
    }
  
    if ( this->verbose ) 
  		cerr << "Transformed the filters in " << stop() << " ms" << endl;

    initw = w;
    inith = h;
  }

  if ( this->verbose )
    start();

  int errcode = this->detect( image.width(), image.height(), pyramid, detections);
 
  if (this->verbose)
    cerr << "Computed the convolutions and distance transforms in " << stop() << " ms" << endl;

  return errcode;
}

int DPMDetection::detect(int width, int height, const HOGPyramid & pyramid, vector<Detection> & detections)
{
  for ( map<std::string, Mixture *>::iterator i = this->mixtures.begin(); i != this->mixtures.end(); i++ )
  {
    Mixture *mixture = i->second;
    const std::string & classname = i->first;
    double threshold = thresholds[classname];

    // Compute the scores
    vector<HOGPyramid::Matrix> scores;
    vector<Mixture::Indices> argmaxes;
    vector<Detection> single_detections;
    
    // there is also a version which allows obtaining the part positions, but who cares :)
    // see the original ffld code for this call
    mixture->convolve(pyramid, scores, argmaxes);
    
    if ( this->verbose )
      cerr << "Running detector for " << classname << endl;
    
    // Cache the size of the models
    vector<pair<int, int> > sizes(mixture->models().size());
    
    for (int i = 0; i < sizes.size(); ++i)
      sizes[i] = mixture->models()[i].rootSize();
    
    // For each scale
    for (int i = pyramid.interval(); i < scores.size(); ++i) {
      // Scale = 8 / 2^(1 - i / interval)
      const double scale = pow(2.0, static_cast<double>(i) / pyramid.interval() + 2.0);
      
      const int rows = scores[i].rows();
      const int cols = scores[i].cols();
      
      for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
          const float score = scores[i](y, x);
          
          if (score > threshold) {
            if (((y == 0) || (x == 0) || (score > scores[i](y - 1, x - 1))) &&
              ((y == 0) || (score > scores[i](y - 1, x))) &&
              ((y == 0) || (x == cols - 1) || (score > scores[i](y - 1, x + 1))) &&
              ((x == 0) || (score > scores[i](y, x - 1))) &&
              ((x == cols - 1) || (score > scores[i](y, x + 1))) &&
              ((y == rows - 1) || (x == 0) || (score > scores[i](y + 1, x - 1))) &&
              ((y == rows - 1) || (score > scores[i](y + 1, x))) &&
              ((y == rows - 1) || (x == cols - 1) || (score > scores[i](y + 1, x + 1)))) {
              FFLD::Rectangle bndbox((int)((x - pyramid.padx()) * scale + 0.5),
                           (int)((y - pyramid.pady()) * scale + 0.5),
                           (int)(sizes[argmaxes[i](y, x)].second * scale + 0.5),
                           (int)(sizes[argmaxes[i](y, x)].first * scale + 0.5));
              
              // Truncate the object
              bndbox.setX(max(bndbox.x(), 0));
              bndbox.setY(max(bndbox.y(), 0));
              bndbox.setWidth(min(bndbox.width(), width - bndbox.x()));
              bndbox.setHeight(min(bndbox.height(), height - bndbox.y()));
              
              if (!bndbox.empty())
                single_detections.push_back(Detection(score, i, x, y, bndbox, classname));

            }
          }
        }
      }
    }

    if ( this->verbose )
      cerr << "Number of detections before non-maximum suppression: " << single_detections.size() << endl;

    
    // Non maxima suppression
    nms( single_detections, this->overlap );

    detections.insert ( detections.begin(), single_detections.begin(), single_detections.end() );
  }

  nms ( detections, this->overlap_all );

  return 0;
}

int DPMDetection::addModels ( const std::string & modellistfn )
{
  ifstream ifs ( modellistfn.c_str(), ifstream::in);

  if ( !ifs.good() )
  {
    cerr << "Unable to open " << modellistfn << endl;
    return -1;
  }

  while ( !ifs.eof() )
  {
    string classname;
    string modelfile;
    double threshold;
    if ( !( ifs >> classname ) ) break;
    if ( !( ifs >> modelfile ) ) break;
    if ( !( ifs >> threshold ) ) break;

    if (classname.substr(0,1) == "#")
      continue;

    cerr << "Adding a model for " << classname << " with threshold " << threshold << " (" << modelfile << ")" << endl;
    if ( addModel ( classname, modelfile, threshold ) < 0 )
    {
      cerr << "Failed to read the model!" << endl;
      continue;
    }
  }

  ifs.close();

  return 0; 

}

void DPMDetection::nms ( std::vector<Detection> & detections, double overlap )
{
  sort(detections.begin(), detections.end());
  
  for (int i = 1; i < detections.size(); ++i)
    detections.resize(remove_if(detections.begin() + i, detections.end(),
                  Intersector(detections[i - 1], overlap, true)) -
              detections.begin());
}


#endif
