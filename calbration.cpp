// calibration danying juzhen

#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <dirent.h>
#include <sys/types.h>

// FOR ROSBAG.
#include <ros/ros.h>
#include <rosbag/player.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;


class Calbration
{
public:
    Calbration ( const std::string& setting_dir ) {
        /* load parameters */
        cv::FileStorage fs ( setting_dir, cv::FileStorage::READ );

        // camera params
        fx = fs["camera.fx"];
        fy = fs["camera.fy"];
        cx = fs["camera.cx"];
        cy = fs["camera.cy"];

        k1 = fs["camera.k1"];
        k2 = fs["camera.k2"];
        p1 = fs["camera.p1"];
        p2 = fs["camera.p2"];
        k3 = fs["camera.k3"];

        K = ( cv::Mat_<float> ( 3, 3 ) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 );
        dist = ( cv::Mat_<float> ( 1, 5 ) << k1, k2, p1, p2, k3 );
        eK << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

        // target params
        target_cols = fs["target.cols"];
        target_rows = fs["target.rows"];
        target_col_grid_size = fs["target.col_grid_size"];
        target_row_grid_size = fs["target.row_grid_size"];
        target_offset = fs["target.offset"];
    }

    bool calibrate ( cv::Mat& image ) {
        /* process the img with one time calibration */

        // extract corners
        std::vector<cv::Point2f> corners, undist_corners;
        bool found = cv::findChessboardCorners ( image, cv::Size ( target_cols, target_rows ), corners );

        if ( found ) {

            // refine corners
            cv::cornerSubPix ( image, corners, cv::Size ( 11, 11 ), cv::Size ( -1, -1 ),
                               cv::TermCriteria ( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1 ) );

            // show corners
            cv::drawChessboardCorners ( image, cv::Size ( target_cols, target_rows ), corners, found );
            cv::resize ( image, image, cv::Size ( image.cols / 4,  image.rows/4 ) );
            cv::imshow ( "detect corners", image );
            cv::waitKey ( 1 );

            // undistort
            cv::undistortPoints ( corners, undist_corners, K, dist, cv::Mat(), K );
        }// if found target
        else {
            return false;
        }

        // get Tct, translation between camera and the target
        // get 3d mappoints refer to the target coordinate
        std::vector<cv::Point3f> pts;
        for ( int i = 0; i < target_rows; ++i )
            for ( int j = 0; j < target_cols; ++j ) {
// 				int yj = ( target_cols - j - 1);
// 				pts.push_back ( cv::Point3f ( yj*target_col_grid_size, i*target_row_grid_size, 0 ) );
				pts.push_back ( cv::Point3f ( i*target_row_grid_size, j*target_col_grid_size, 0 ) );
            }

        cv::Vec3d rvec, tvec;
        bool solve_ok =  cv::solvePnP ( pts, undist_corners, K, cv::Mat(), rvec, tvec );
        if ( !solve_ok ) {
            return false;
        }

        // convert to homo T
        Eigen::Matrix3d R;
        cv::Mat cvR;
        cv::Rodrigues ( rvec, cvR );
        R <<  cvR.at<double> ( 0, 0 ), cvR.at<double> ( 0, 1 ), cvR.at<double> ( 0, 2 ),
          cvR.at<double> ( 1, 0 ), cvR.at<double> ( 1, 1 ), cvR.at<double> ( 1, 2 ),
          cvR.at<double> ( 2, 0 ), cvR.at<double> ( 2, 1 ), cvR.at<double> ( 2, 2 );

        Eigen::Vector3d t;
        t << tvec[0], tvec[1], tvec[2];

        Eigen::Vector3d r1 = R.block ( 0, 0, 3, 1 );
        Eigen::Vector3d r2 = R.block ( 0, 1, 3, 1 );

        Eigen::Vector3d tb;
        tb << 0.0, 0.0, target_offset;

        Eigen::Matrix3d RT;
        RT.block ( 0,0, 3, 1 ) = r1;
        RT.block ( 0,1, 3, 1 ) = r2;
        RT.block ( 0,2, 3, 1 ) = R * tb + t;

        H = eK * RT; // homography
        std::cout << std::endl << H << std::endl << std::endl << std::endl;
        // Save result.
        return true;
    }

    void saveResult ( const std::string& result_dir ) {
        cv::FileStorage fs ( result_dir, cv::FileStorage::WRITE );

        // Save homography Matrix
        cv::Mat cvH = ( cv::Mat_<double> ( 3, 3 ) << H ( 0,0 ), H ( 0,1 ), H ( 0,2 ), H ( 1,0 ), H ( 1,1 ), H ( 1,2 ), H ( 2,0 ), H ( 2,1 ), H ( 2,2 ) );
        fs << "homograph_matrix" << cvH;
        fs.release();
    }

private:

    float fx;
    float fy;
    float cx;
    float cy;

    float k1;
    float k2;
    float p1;
    float p2;
    float k3;
    cv::Mat K;
    cv::Mat dist;
    Eigen::Matrix3d eK;

    int target_cols;
    int target_rows;
    float target_col_grid_size;
    float target_row_grid_size;
    float target_offset;

    Eigen::Matrix3d H;
};// class Calbration


int main ( int argc, char **argv )
{
    if ( argc != 4 ) {
        std::cout << "Please input: setting file, rosbag, homograph_matrix.yaml\n";
        return -1;
    }

    /* load parameters */
    const std::string cfg_dir = argv[1];
    const std::string rosbag_dir = argv[2];
    const std::string calibration_dir = argv[3];

    // Init Calbration class.
    Calbration calibration ( cfg_dir );

    // Read ROS bag.
    const std::string img_topic = "/daheng/gray";
    rosbag::Bag bag;
    bag.open ( rosbag_dir, rosbag::bagmode::Read );
    std::vector<std::string>  topics;
    topics.push_back ( img_topic );
    rosbag::View viewer ( bag, rosbag::TopicQuery ( topics ) );
    foreach ( rosbag::MessageInstance const m, viewer ) {
        sensor_msgs::Image::ConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
        if ( img_ptr != nullptr ) {
            cv_bridge::CvImageConstPtr cvptr = cv_bridge::toCvShare ( img_ptr );
            cv::Mat img = cvptr->image;

            // Calibration.
            calibration.calibrate ( img );
            calibration.saveResult ( calibration_dir );
            
            cv::waitKey( 0 );
        } // is is a good topic
    }// for all images


    std::cout << "Complete" << std::endl;
    return 0;
}

