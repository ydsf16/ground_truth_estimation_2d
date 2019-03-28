#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

// FOR ROSBAG USAGES.
#include <ros/ros.h>
#include <rosbag/player.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


class Estimation
{
public:
    Estimation ( const std::string& setting_dir, const std::string& calbration_dir ) {
        // local parameters
        cv::FileStorage fs ( setting_dir, cv::FileStorage::READ );
        cv::FileStorage fs_homography ( calbration_dir, cv::FileStorage::READ );

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

        // aruco marker params
        n_markers = fs["aruco.n_markers"];
        marker_size = fs["aruco.marker_size"];
        marker_length = fs["aruco.marker_length"];

        // Homograph matrix
        cv::Mat cvH ;
        fs_homography["homograph_matrix"] >> cvH;
        H << cvH.at<double> ( 0, 0 ), cvH.at<double> ( 0, 1 ), cvH.at<double> ( 0, 2 ),
          cvH.at<double> ( 1, 0 ), cvH.at<double> ( 1, 1 ), cvH.at<double> ( 1, 2 ),
          cvH.at<double> ( 2, 0 ), cvH.at<double> ( 2, 1 ), cvH.at<double> ( 2, 2 );
        H_inv = H.inverse();

        // init aruco dictionary
        dictionary = cv::aruco::generateCustomDictionary ( n_markers, marker_size );
    }

    bool estimate ( cv::Mat& image, Eigen::Vector2d& xy, Eigen::Quaterniond& q ) {

        cv::cvtColor ( image, image, cv::COLOR_GRAY2BGR );

        // TODO to be changed for gt.
        //cv::circle(image, cv::Point2d(580, 710), 35, cv::Scalar(0, 0, 0), -1);
        
        // detect aruco marker
        std::vector< std::vector<cv::Point2f> > marker_corners;
        std::vector<int> IDs;
        cv::aruco::detectMarkers ( image, dictionary , marker_corners, IDs );
        cv::aruco::drawDetectedMarkers ( image, marker_corners, IDs );
        
        cv::Mat dst;
        cv::resize(image, dst, cv::Size(image.cols /2 , image.rows / 2));
        cv::imshow("dst", dst);
        cv::waitKey(1);
        
        if ( IDs.size() != 1 ) {
            return false;
        }

        // get the four corners
        std::vector<Eigen::Vector2d> pts;
        std::vector< cv::Point2f >& corners = marker_corners.at ( 0 );

        // undist
        std::vector<cv::Point2f> undist_corners;
        cv::undistortPoints ( corners, undist_corners, K, dist, cv::Mat(), K );


        for ( size_t np = 0; np < undist_corners.size(); np ++ ) {
            cv::Point2f& corner = undist_corners.at ( np );
            Eigen::Vector3d e_corner ( corner.x, corner.y, 1.0 );
            Eigen::Vector3d pt = H_inv * e_corner;
            pt = pt / pt[2];
            pts.push_back ( Eigen::Vector2d ( pt[0], pt[1] ) );
        }

        // check for corners
        double d1 = ( pts[0] - pts[1] ).norm();
        double d2 = ( pts[1] - pts[2] ).norm();
        double d3 = ( pts[2] - pts[3] ).norm();
        double d4 = ( pts[3] - pts[0] ).norm();
        double d_mean = ( d1 + d2 + d3 + d4 ) *0.25;
        if ( fabs ( ( d_mean - marker_length ) / marker_length ) > 0.2 ) {
            std::cout << "Bad length" << std::endl;
            return false;
        }

        // calc theta
        Eigen::Vector2d pta = pts[0] - pts[3];
        Eigen::Vector2d ptb = pts[1] - pts[2];
        double theta1 = atan2 ( pta[1], pta[0] );
        double theta2 = atan2 ( ptb[1], ptb[0] );
        double d_theta = theta1 -theta2;
        double theta = 0.0;
        if ( fabs ( d_theta ) > M_PI ) {
            if ( d_theta >  M_PI ) {
                d_theta -= 2.0 * M_PI;
            }
            if ( d_theta <= -M_PI ) {
                d_theta += 2.0 * M_PI;
            }
            if ( fabs ( d_theta ) < 0.03 ) {
                theta = M_PI;
            } else {
                std::cout << "Bad theta\n";
                return false;
            }
        } else if ( fabs ( d_theta < 0.03 ) ) {
            theta = 0.5 * ( theta1 + theta2 );
        } else {
            std::cout << "Bad theta\n";
        }

        // offset TODO.
       // const double DTHETA = M_PI / 2.0;
       // theta += DTHETA;

        if ( theta >  M_PI ) {
            theta -= 2.0 * M_PI;
        }
        if ( theta <= -M_PI ) {
            theta += 2.0 * M_PI;
        }

        if ( init == false ) {
            last_theta = theta;
            init = true;
            return false;
        } else {
            double dist_theta = theta - last_theta;
            last_theta = theta;
            
            if ( dist_theta >  M_PI ) {
                dist_theta -= 2.0 * M_PI;
            }
            if ( dist_theta <= -M_PI ) {
                dist_theta += 2.0 * M_PI;
            }

            dist_theta = fabs ( dist_theta );

            if ( dist_theta > 0.5 ) {
                return false;
            } 
        }

        xy = ( pts[0] + pts[1] + pts[2] + pts[3] ) * 0.25;
        // std::cout << " " << xy[0] << " " << xy[1] << " " << theta << "\n";

        // save as TUM format
        Eigen::Matrix3d R;
        R << cos ( theta ), -sin ( theta ), 0.0,
          sin ( theta ), cos ( theta ), 0.0,
          0.0, 0.0, 1.0;
        Eigen::Quaterniond qx ( R );
        q = qx;

        return true;
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

    int n_markers;
    int marker_size;
    double marker_length;


    Eigen::Matrix3d H;
    Eigen::Matrix3d H_inv;

    cv::Ptr<cv::aruco::Dictionary> dictionary;

    bool init = false;
    double last_theta = 0.0;
};



int main ( int argc, char **argv )
{

    if ( argc != 5 ) {
        std::cout << "Please input: setting.yaml, homograph_matrix.yaml, rosbag, result.txt\n";
        return -1;
    }

    /* load parameters */
    const std::string cfg_dir = argv[1];
    const std::string h_matrix_dir = argv[2];
    const std::string rosbag_dir = argv[3];
    const std::string result_dir = argv[4];

    // Init estimation.
    Estimation estimation ( cfg_dir, h_matrix_dir );
    std::ofstream  groundtruth_file;
    groundtruth_file.open ( result_dir );
    groundtruth_file << std::fixed;
    std::cout << std::fixed;

    // Read ROS bag.
    const std::string img_topic = "/daheng/gray"; // TODO to be changed.
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
            double timestamp = cvptr->header.stamp.toSec();

            // Estimation.
            Eigen::Vector2d xy;
            Eigen::Quaterniond q;
            if ( estimation.estimate ( img, xy, q ) ) {
                // save to file.
                groundtruth_file << std::setprecision ( 6 ) << timestamp << " " <<  std::setprecision ( 9 ) << xy[0] << " " << xy[1] << " " << 0.0 << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
                std::cout << std::setprecision ( 6 ) << timestamp << " " <<  std::setprecision ( 9 ) << xy[0] << " " << xy[1] << " " << 0.0 << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
            }
        } // if is a good topic
    }// for all images

    return 0;
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
