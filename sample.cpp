#include<opencv2/opencv.hpp>
#include "raspivideocap.h"

using namespace std;
using namespace cv;

int test( )
{
  RaspiVideoCapture cap();
 
  if( !cap.open( 640, 480, 30 ) )
  {
      fprintf( stderr, "Failed to open camera\n" );
      return 1;
  }


  for(int i=0;i<10;i++) {
    cv::Mat img_in;
    cap.read( img_in );
    cv::imwrite("sample.jpg",img_in);
  }


  return 0;
}