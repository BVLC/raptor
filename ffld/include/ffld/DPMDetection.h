/**
 * @author: Erik Rodner
 * 
*/

#include <string>
#include <map>

#include "ffld/HOGPyramid.h"
#include "ffld/Mixture.h"
#include "ffld/Patchwork.h"
#include "ffld/JPEGImage.h"

namespace FFLD {

/** single detection result */
struct Detection : public FFLD::Rectangle
{
	FFLD::HOGPyramid::Scalar score;
	int l;
	int x;
	int y;
  std::string classname;
	
	Detection() : score(0), l(0), x(0), y(0)
	{
	}
	
	Detection(FFLD::HOGPyramid::Scalar score, int l, int x, int y, FFLD::Rectangle bndbox, const std::string & classname) :
	FFLD::Rectangle(bndbox), score(score), l(l), x(x), y(y), classname(classname)
	{
	}
	
	bool operator<(const Detection & detection) const
	{
		return score > detection.score;
	}
};

/** detection class providing the same functionality as the ffld binary */
class DPMDetection
{
  public:

    DPMDetection ( bool verbose = false, double overlap = 0.5, double overlap_all = 0.5, int padding = 12, int interval = 10 ); 

    /** load a single model from disk */
    DPMDetection ( const std::string & modelfile, double threshold = 0.8, bool verbose = false, double overlap = 0.5, double overlap_all = 0.5, int padding = 12, int interval = 10 );

    ~DPMDetection();

    int detect ( const JPEGImage & image, std::vector<Detection> & detections );
    
    int addModel ( const std::string & classname, const std::string & modelfile, double threshold );

    int addModels ( const std::string & modellistfn );

  private:
    double overlap;
    double overlap_all;
    int padding;
    int interval;
    bool verbose;
    int initw;
    int inith;

    std::map<std::string, Mixture *> mixtures;
    std::map<std::string, double> thresholds;

    void init ( bool verbose, double overlap, double overlap_all, int padding, int interval );
    
    int detect( int width, int height, const HOGPyramid & pyramid, std::vector<Detection> & detections );

    void nms ( std::vector<Detection> & detections, double overlap );
};


}
