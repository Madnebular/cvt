#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include <cvt/io/Resources.h>

#include <cvt/util/Exception.h>
#include <cvt/util/Timer.h>
#include <cvt/math/SparseBundleAdjustment.h>
#include <cvt/math/SBAData.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cv.h>
#include <cvaux.hpp>

// test app for sparse bundle adjustment, 
// test input files from the sba-lib:
struct Point3d 
{	
	Point3d( double x, double y, double z ):
		P( x, y, z )
	{		
	}
			
	Eigen::Vector3d P;
	std::vector<Eigen::Vector2d> observations;
	std::vector<size_t> frameIds;
};

struct CameraTransform {
	Eigen::Quaterniond orientation;
	Eigen::Vector3d	 translation;
};

void parseCalibFile( const std::string & file, Eigen::Matrix3d & K )
{
	std::ifstream fileIn( file.c_str() );
	std::istringstream lineBuffer;

	double val;
	size_t row = 0;
	size_t col = 0;

	std::string line;
	while ( std::getline( fileIn, line ) ){
		lineBuffer.clear();
		lineBuffer.str( line );
		col = 0;
		while ( lineBuffer >> val ){
			K( row, col++ ) = val;
		}
		row++;
	}	
	
	fileIn.close();	
}

void parseCameraFile( const std::string & file, std::vector<CameraTransform> & cameras )
{
	std::ifstream fileIn( file.c_str() );
	std::istringstream lineBuffer;
	
	double val;
	
	CameraTransform cam;
	
	std::string line;
	while ( std::getline( fileIn, line ) ){
		lineBuffer.clear();
		lineBuffer.str( line );
		
		lineBuffer >> val; cam.orientation.w() = val;
		lineBuffer >> val; cam.orientation.x() = val;
		lineBuffer >> val; cam.orientation.y() = val;
		lineBuffer >> val; cam.orientation.z() = val;
		lineBuffer >> val; cam.translation.x() = val;
		lineBuffer >> val; cam.translation.y() = val;
		lineBuffer >> val; cam.translation.z() = val;
		
		cameras.push_back( cam );
	}	
	
	fileIn.close();	
}

void parsePtsFile( const std::string & file, std::vector<Point3d> & points3d )
{
	std::ifstream fileIn( file.c_str() );
	std::istringstream lineBuffer;
	
	double x, y, z;
	size_t frameId;
	
	unsigned int numFrames;
	unsigned int currentLine = 0;
	
	std::string line;
	
	// step over first line
	std::getline( fileIn, line );
	while ( std::getline( fileIn, line ) ){
		currentLine++;

		lineBuffer.clear();
		lineBuffer.str( line );
		
		lineBuffer >> x; lineBuffer >> y; lineBuffer >> z;
		points3d.push_back( Point3d( x, y, z ) );
		
		lineBuffer >> numFrames;		
		while (numFrames--) {
			lineBuffer >> frameId;
			lineBuffer >> x; lineBuffer >> y;
			
			points3d.back().observations.push_back( Eigen::Vector2d( x, y ) );
			points3d.back().frameIds.push_back( frameId );
		}
	}	
	
	fileIn.close();	
}

void loadGroundTruth( const std::string & cams, 
					  const std::string & pts, 
					  std::vector<Eigen::Matrix3d> & rotations, 
					  std::vector<Eigen::Vector3d> & translations,
					  std::vector<Eigen::Vector3d> & points )
{
	std::ifstream camFile( cams.c_str() );
	std::istringstream lineBuffer;
		
	Eigen::Quaterniond q;
		
	std::string line;

	// skip first line
	std::getline( camFile, line );

	while ( std::getline( camFile, line ) ){
		lineBuffer.clear();
		lineBuffer.str( line );
		
		lineBuffer >> q.w();
		lineBuffer >> q.x();
		lineBuffer >> q.y();
		lineBuffer >> q.z();
		rotations.push_back( q.toRotationMatrix() );
		
		translations.push_back( Eigen::Vector3d::Zero() );
		lineBuffer >> translations.back()(0);
		lineBuffer >> translations.back()(1);
		lineBuffer >> translations.back()(2);
	}	
	camFile.close();	
	
	std::ifstream ptsFile( pts.c_str() );
	std::getline( ptsFile, line );
	
	while ( std::getline( ptsFile, line ) ){
		lineBuffer.clear();
		lineBuffer.str( line );
		
		points.push_back( Eigen::Vector3d::Zero() );
				
		lineBuffer >> points.back()(0);
		lineBuffer >> points.back()(1);
		lineBuffer >> points.back()(2);
	}	
	ptsFile.close();	
}

void printLoadedData( Eigen::Matrix3d & K, 
					  std::vector<CameraTransform> & cameras, 
					  std::vector<Point3d> & points3d )
{
	std::cout << "Intrinsics:\n" << K << std::endl;
	
	for(unsigned int i = 0; i < cameras.size(); i++){
		std::cout << "Camera " << i << std::endl;
		std::cout << "\tOrientation: " 
		<< cameras[ i ].orientation.w() << ", " 
		<< cameras[ i ].orientation.x() << ", "
		<< cameras[ i ].orientation.y() << ", "
		<< cameras[ i ].orientation.z() << std::endl;
		
		std::cout << "\tTranslation" << cameras[ i ].translation( 0 ) << ", " 
									 << cameras[ i ].translation( 1 ) << ", " 
									 << cameras[ i ].translation( 2 ) << std::endl;
	}
	
	std::cout << "Observations for 3D Points:" << std::endl;
	for(unsigned int i = 0; i < points3d.size(); i++){
		std::cout << "3D Point:" << points3d[ i ].P.x() << ", " << points3d[ i ].P.y() << ", " << points3d[ i ].P.z() << std::endl;
		
		for(unsigned int k = 0; k < points3d[ i ].observations.size(); k++){
			std::cout << "\tId: " << points3d[ i ].frameIds[ k ];
			std::cout << "\tp: " << points3d[ i ].observations[ k ].x() << ", " << points3d[ i ].observations[ k ].y() << std::endl;
		}
	}
}

void convertDataToOpenCv(Eigen::Matrix3d & K, 
						 std::vector<CameraTransform> & cameras, 
						 std::vector<Point3d> & points3d,
						 cv::vector<cv::Point3d> & pointsOCV, /* 3d points */
						 cv::vector<cv::vector<cv::Point2d> > & imagePoints, /* projections of every point for every camera */
						 cv::vector<cv::vector<int> > & visibility,/* visibility of every 3d point for every camera */
						 cv::vector<cv::Mat> & intrinsics, /* for each cam*/
						 cv::vector<cv::Mat> & rotations,
						 cv::vector<cv::Mat> & translations,
						 cv::vector<cv::Mat> & distortions )
{
	imagePoints.resize( cameras.size() );
	visibility.resize( cameras.size() );

	for( unsigned int i = 0; i < imagePoints.size(); i++ ) {
		imagePoints[ i ].resize( points3d.size() );
		visibility[ i ].resize( points3d.size(), 0 );
	}
	
	cv::Point3d currentPoint;
	for(unsigned int i = 0; i < points3d.size(); i++){
		currentPoint.x = points3d[ i ].P.x();
		currentPoint.y = points3d[ i ].P.y();
		currentPoint.z = points3d[ i ].P.z();
		pointsOCV.push_back( currentPoint );
		
		for( unsigned int p = 0; p < points3d[i].observations.size(); p++ ){
			imagePoints[ points3d[ i ].frameIds[ p ] ][ p ].x = points3d[ i ].observations[ p ].x();
			imagePoints[ points3d[ i ].frameIds[ p ] ][ p ].y = points3d[ i ].observations[ p ].y();
			
			visibility[ points3d[ i ].frameIds[ p ] ][ p ] = 1; // visible
		}
	}
		
	cv::Mat intr( 3, 3, CV_64F );
	cv::Mat rot( 3, 3, CV_64F );
	cv::Mat trans( 3, 1, CV_64F );
	cv::Mat dist = cv::Mat::zeros( 5, 1, CV_64F );
	
	for(unsigned int r = 0; r < 3; r++)
		for(unsigned int c = 0; c < 3; c++)
			intr.at<double>( r, c ) = K( r, c );
	
	// fill initial camera estimates:
	Eigen::Matrix3d rotEI;
	for(unsigned int i = 0; i < cameras.size(); i++){
		// same intrinsics for all
		intrinsics.push_back( intr );		
		distortions.push_back( dist );
		
		// rotation init:
		rotEI = cameras[ i ].orientation.toRotationMatrix();
		for(unsigned int r = 0; r < 3; r++)
			for(unsigned int c = 0; c < 3; c++)
				rot.at<double>( r, c ) = rotEI( r, c );
		rotations.push_back( rot );
			
		// translation init:
		trans.at<double>( 0, 0 ) = cameras[ i ].translation.x();
		trans.at<double>( 1, 0 ) = cameras[ i ].translation.y();
		trans.at<double>( 2, 0 ) = cameras[ i ].translation.z();
		translations.push_back( trans );
	}
}

void convertData(Eigen::Matrix3d & K, 
				 std::vector<CameraTransform> & cameras, 
				 std::vector<Point3d> & points3d,
				 cvt::SBAData & sbaData)
{
	std::vector<cvt::KeyFrame*> keyFrames( cameras.size() );
	
	for( unsigned int i = 0; i < cameras.size(); i++ ){
		keyFrames[ i ] = new cvt::KeyFrame( K, i );
		cvt::CameraModel & cam =  keyFrames[ i ]->camera;
		cam.set( cameras[ i ].orientation, cameras[ i ].translation );
		cam.updateProjectionMatrix();
		
		sbaData.addKeyFrame( keyFrames[ i ] );
	}
	
	for( unsigned int i = 0; i < points3d.size(); i++ ){
		Eigen::Vector4d p;
		p.block(0, 0, 3, 1) = points3d[ i ].P; 
		p(3)=1.0;
		size_t id;
		
		Eigen::Vector4d * point = sbaData.addPoint( p, id );
		
		size_t view;
		
		for( unsigned int v = 0; v < points3d[ i ].frameIds.size(); ++v ){
			view = points3d[ i ].frameIds[ v ];
			
			keyFrames[ view ]->addMeasurement( point, id, points3d[ i ].observations[ v ] );
			
			sbaData.addFrameIdForPoint(id, view );	
		}					
	}	
	sbaData.updateCosts();
	
//	std::cout << *(sbaData.frames()[0]->measurements[0].point3d->data) << std::endl;

}

void testOpenCVImplementation( Eigen::Matrix3d & K, 
							   std::vector<CameraTransform> & cameras, 
							   std::vector<Point3d> & points3d )
{
	cv::vector<cv::Point3d> pointsOCV;
	cv::vector<cv::vector<cv::Point2d> > imagePoints;
	cv::vector<cv::vector<int> > visibility;
	cv::vector<cv::Mat> intrinsics;
	cv::vector<cv::Mat> rotations;
	cv::vector<cv::Mat> translations;
	cv::vector<cv::Mat> distortions;
	
	convertDataToOpenCv( K, cameras, points3d,
						 pointsOCV, imagePoints, visibility, 
						 intrinsics, rotations, translations, distortions );
	
	cv::TermCriteria criteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON );
		
	cvt::Timer timer;
		
	std::cout << pointsOCV.size() << std::endl;
	cv::LevMarqSparse::bundleAdjust( pointsOCV,
									 imagePoints,
									 visibility,
									 intrinsics,
									 rotations, 
									 translations,
									 distortions,
									 criteria );
	
	std::cout << "OpenCV BA took: " << timer.elapsedMiliSeconds() << "ms" << std::endl;	
}

void showResults( cvt::SBAData & sbaData )
{	
	// Cameras:
	for( unsigned int i = 0; i < sbaData.frames().size(); i++ ){
		std::cout << "Camera " << i << std::endl;		
		Eigen::Quaterniond q( sbaData.frames()[ i ]->camera.R() );
		const Eigen::Vector3d & t = sbaData.frames()[ i ]->camera.t();
		std::cout << "\tRotation: [ " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]" << std::endl;
		std::cout << "\tTranslation: [ " << t.x() << ", " << t.y() << ", " << t.z() << "]" << std::endl;
		std::cout << std::endl;
	}
	
	sbaData.printPoints();
}

void genPts( const Eigen::Vector3d & min, const Eigen::Vector3d & max, size_t num, std::vector<Eigen::Vector3d> & pts )
{
	// random numbers
	cv::RNG rng( time( NULL ) );
	
	Eigen::Vector3d curr;
	for( size_t i = 0; i < num; i++ ){
		curr( 0 ) = rng.uniform( min( 0 ), max( 0 ));
		curr( 1 ) = rng.uniform( min( 1 ), max( 1 ));
		curr( 2 ) = rng.uniform( min( 2 ), max( 2 ));
		
		pts.push_back( curr );
	}
}

void genCams( const Eigen::Matrix<double, 6, 1> & min, const Eigen::Matrix<double, 6, 1> & max, size_t num, 
			  std::vector<Eigen::Matrix3d> & R, std::vector<Eigen::Vector3d> & t )
{	
	cv::RNG rng( time( NULL ) );
	
	Eigen::Vector3d angles;
	Eigen::Matrix3d rTmp;
	Eigen::Vector3d trans;
	for( size_t i = 0; i < num; i++ )
	{
		angles( 0 ) = rng.uniform( min( 0 ), max( 0 ) );
		angles( 1 ) = rng.uniform( min( 1 ), max( 1 ) );
		angles( 2 ) = rng.uniform( min( 2 ), max( 2 ) );
		
		// calc rotMat ...
		rTmp = (Eigen::AngleAxisd( angles( 0 ), Eigen::Vector3d::UnitX() ) * 
			   Eigen::AngleAxisd( angles( 1 ), Eigen::Vector3d::UnitY() ) * 
			   Eigen::AngleAxisd( angles( 2 ), Eigen::Vector3d::UnitZ() )).toRotationMatrix();
		
		R.push_back( rTmp );		
		
		trans( 0 ) = rng.uniform( min( 3 ), max( 3 ));
		trans( 1 ) = rng.uniform( min( 4 ), max( 4 ));
		trans( 2 ) = rng.uniform( min( 5 ), max( 5 ));	
		t.push_back( trans );
	}
}

void simulateData( cvt::SBAData & sbaData,
				   std::vector<Eigen::Matrix3d> & rot,
				   std::vector<Eigen::Vector3d> & trans,
				   std::vector<Eigen::Vector3d> & pts )
{
	
	size_t numPoints = 200;
	size_t numCams = 10;
	
	Eigen::Vector3d minP, maxP;
	
	minP( 0 ) = minP( 1 ) = minP( 2 ) = 0.0;
	maxP( 0 ) = maxP( 1 ) = maxP( 2 ) = 100.0;	
	genPts( minP, maxP, numPoints, pts );
		
	Eigen::Matrix<double, 6, 1> minC, maxC;
	minC( 0 ) = minC( 1 ) = minC( 2 ) = -M_PI / 4.0;
	maxC( 0 ) = maxC( 1 ) = maxC( 2 ) = M_PI / 4.0;	
	minC( 3 ) = minC( 4 ) = minC( 5 ) = 100.0;
	maxC( 3 ) = maxC( 4 ) = maxC( 5 ) = 110.0;
	genCams( minC, maxC, numCams, rot, trans );
	
	Eigen::Matrix3d K( Eigen::Matrix3d::Identity() );
	K( 0, 0 ) = 640.0;
	K( 1, 1 ) = 638.0;
	K( 0, 2 ) = 320.0;
	K( 1, 2 ) = 240.0;
	
	// setup the keyframes:
	cvt::KeyFrame * frame;
	cv::RNG rng( time( NULL ) );
	Eigen::Matrix3d dRot, R;
	Eigen::Vector3d dt, t;
	double alpha, beta, gamma;
		
	const double angleNoise = 3.0;
	const double transNoise = 3.0;
	
	for( size_t c = 0; c < numCams; c++ ){
		alpha = rng.uniform( -angleNoise * M_PI / 180.0, angleNoise * M_PI / 180.0 );
		beta = rng.uniform( -angleNoise * M_PI / 180.0, angleNoise * M_PI / 180.0 );
		gamma = rng.uniform( -angleNoise * M_PI / 180.0, angleNoise * M_PI / 180.0 );
		
		dRot = (Eigen::AngleAxisd( alpha, Eigen::Vector3d::UnitX() ) * 
				Eigen::AngleAxisd( beta, Eigen::Vector3d::UnitY() ) * 
				Eigen::AngleAxisd( gamma, Eigen::Vector3d::UnitZ() )).toRotationMatrix();
		R = rot[ c ] * dRot;
		
		dt( 0 ) = rng.uniform( -transNoise, transNoise );
		dt( 1 ) = rng.uniform( -transNoise, transNoise );
		dt( 2 ) = rng.uniform( -transNoise, transNoise );
		
		t = trans[ c ] + dt;
		
		frame = new cvt::KeyFrame( K, c );
		frame->camera.set( R, t );
		frame->camera.updateProjectionMatrix();
		
		sbaData.addKeyFrame( frame );		
	}
	
	// now add the points:
	const double pNoise = 3.0;
	Eigen::Vector4d p;
	Eigen::Vector3d pCam;
	Eigen::Vector2d imgP;
	for( size_t i = 0; i < numPoints; i++ ){
		p.block(0, 0, 3, 1) = pts[ i ];
		
		// add noise to gt 3d point as initialization
		p( 0 ) = pts[ i ]( 0 ) + rng.uniform( -pNoise, pNoise );
		p( 1 ) = pts[ i ]( 1 ) + rng.uniform( -pNoise, pNoise );
		p( 2 ) = pts[ i ]( 2 ) + rng.uniform( -pNoise, pNoise );		
		p( 3 ) = 1.0;
		
		size_t id;		
		Eigen::Vector4d * point = sbaData.addPoint( p, id );
		
		for( size_t c = 0; c < numCams; c++ ){
			pCam = ( rot[ c ] * pts[ i ]) + trans[ c ];
			
			if( pCam.z() > 2.0 ){				
				pCam = K * pCam;
				imgP.x() = pCam.x() / pCam.z();
				imgP.y() = pCam.y() / pCam.z();				
								
				// point is visible in this view:
				sbaData.frames()[ c ]->addMeasurement( point, id, imgP );						
				sbaData.addFrameIdForPoint( id, c );
					
				//std::cout << "Point " << id << " visible in View " << c << std::endl;
				//std::cout << "[ " << imgP.x() << ", " << imgP.y() << " ]" << std::endl;
			}
		}
	}
	sbaData.updateCosts();	
}

void compareResults( cvt::SBAData & sbaData, 
					 std::vector<Eigen::Matrix3d> & rot,
					 std::vector<Eigen::Vector3d> & trans,
					 std::vector<Eigen::Vector3d> & pts )
{
	// TODO: 
	// compare the results against the result 
	// from the sba package or GT data in simulation case
	
	Eigen::Vector3d tmpVec;
	Eigen::Matrix3d tmpMat;
	
	double RDiff = 0.0;
	double TDiff = 0.0;
	double n, gtn;
	for( unsigned int i = 0; i < rot.size(); i++ ){
		cvt::CameraModel & cam = sbaData.cameraWithId( i );
		
		// compare rotation matrices
		tmpMat = ( cam.R() - rot[ i ] );
		
		/*
		std::cout << "Cam " << i << " GT -> EST " << std::endl;
		std::cout << cam.R()(0, 0) << " " << cam.R()(0, 1) << " " << cam.R()(0, 2) << "\t\t" << rot[i](0, 0) << " " << rot[i](0, 1) << " " << rot[i](0, 2) << std::endl;
		std::cout << cam.R()(1, 0) << " " << cam.R()(1, 1) << " " << cam.R()(1, 2) << "\t\t" << rot[i](1, 0) << " " << rot[i](1, 1) << " " << rot[i](1, 2) << std::endl;
		std::cout << cam.R()(2, 0) << " " << cam.R()(2, 1) << " " << cam.R()(2, 2) << "\t\t" << rot[i](2, 0) << " " << rot[i](2, 1) << " " << rot[i](2, 2) << std::endl;
		std::cout << std::endl;
		 */
		
		for( unsigned int r = 0; r < 3; r++ ){
			for( unsigned int c = 0; c < 3; c++ ){
				RDiff += (tmpMat( r, c ) * tmpMat( r, c ));
			}			
		}		
		
		// compare translations
		tmpVec = ( cam.t() - trans[ i ] );
		n = tmpVec.norm();		
		
		TDiff += n;
	}
	
	double pointDiff = 0.0;	
	for( unsigned int i = 0; i < pts.size(); i++ ){
		// calc point error (difference between the two points)
		const Eigen::Vector4d & p4 = sbaData.pointWithId( i );
		tmpVec(0) =  p4( 0 ) / p4( 3 ) - pts[i]( 0 ); 
		tmpVec(1) =  p4( 1 ) / p4( 3 ) - pts[i]( 1 );
		tmpVec(2) =  p4( 2 ) / p4( 3 ) - pts[i]( 2 );
		pointDiff += tmpVec.norm();
		
		/*
		std::cout << "Point ( GT -> EST ) " << i << std::endl;
		std::cout << "\t" << pts[ i ]( 0 ) <<" -> " << p4( 0 ) << std::endl; 
		std::cout << "\t" << pts[ i ]( 1 ) <<" -> " << p4( 1 ) << std::endl; 
		std::cout << "\t" << pts[ i ]( 2 ) <<" -> " << p4( 2 ) << std::endl; 
		std::cout << std::endl;
		 */
	}
	std::cout << "Average Cam SSD: R -> " << RDiff / rot.size() << " t -> " << TDiff / trans.size() << std::endl;
	std::cout << "Average P SSD: " << pointDiff / pts.size() << std::endl;
	
}

void testMVGImplementation( Eigen::Matrix3d & K, 
						    std::vector<CameraTransform> & cameras, 
						    std::vector<Point3d> & points3d,
						    std::vector<Eigen::Matrix3d> & rot,
						    std::vector<Eigen::Vector3d> & trans,
						    std::vector<Eigen::Vector3d> & pts )
{
	cvt::SBAData sbaData;
	cvt::SparseBundleAdjustment sba;
	sba.setTerminationCriteria( 1e-12, 30 );
	
	convertData( K, cameras, points3d, sbaData );
	
	cvt::Timer timer;
	sba.optimize( sbaData );
	
	std::cout << "MVG BA took: " << timer.elapsedMiliSeconds() << "ms" << std::endl;	
	std::cout << "Iterations: " << sba.iterations() <<", Final epsilon: " << sba.epsilon() << std::endl; 
	std::cout << "Time/iteration: " << timer.elapsedSeconds() / sba.iterations() << std::endl;
	
	//std::cout << "Results" << std::endl;
	//showResults( sbaData );	
	compareResults( sbaData, rot, trans, pts );
}

void testSimulation()
{
	cvt::SBAData sbaData;
	std::vector<Eigen::Matrix3d> rot;
	std::vector<Eigen::Vector3d> trans;
	std::vector<Eigen::Vector3d> pts;
	
	simulateData( sbaData, rot, trans, pts );
	
	cvt::SparseBundleAdjustment sba;
	sba.setTerminationCriteria( 1e-10, 40 );

	cvt::Timer timer;
	
	sba.optimize( sbaData );
	
	double elapsedSecs = timer.elapsedSeconds();
	
	std::cout << "MVG BA took: " << elapsedSecs*1000.0 << "ms" << std::endl;	
	std::cout << "Iterations: " << sba.iterations() <<", Final epsilon: " << sba.epsilon() << std::endl; 
	std::cout << "Time/iteration: " << elapsedSecs / sba.iterations() << std::endl;
	
	compareResults( sbaData, rot, trans, pts );
}

int main(int argc, char* argv[])
{
	cvt::Resources resources;
	
	Eigen::Matrix3d K;
	std::vector<CameraTransform> cameras;
	std::vector<Point3d> points3d;
		
	try {
		std::string intrinsicsFile = resources.find("sba/calib.txt");
				
		//std::string camFile = resources.find("sba/7cams.txt");
		//std::string camFile = resources.find("sba/7camsvarK.txt");
		//std::string pointFile = resources.find("sba/7pts.txt");
		//std::string camGT = resources.find("sba/resultCams7.txt");
		//std::string pointGT = resources.find("sba/resultPts7.txt");
		
		//std::string camFile = resources.find("sba/9cams.txt");
		//std::string camFile = resources.find("sba/9camsvarK.txt");
		//std::string pointFile = resources.find("sba/9pts.txt");
		//std::string camGT = resources.find("sba/resultCams9.txt");
		//std::string pointGT = resources.find("sba/resultPts9.txt");
		
		//std::string camFile = resources.find("sba/54cams.txt");
		//std::string camFile = resources.find("sba/54camsvarK.txt");
		//std::string camFile = resources.find("sba/54camsvarKD.txt");
		//std::string pointFile = resources.find("sba/54pts.txt");
		//std::string camGT = resources.find("sba/resultCams54.txt");
		//std::string pointGT = resources.find("sba/resultPts54.txt");
		
		std::vector<Eigen::Matrix3d> gtRots;
		std::vector<Eigen::Vector3d> gtTrans;
		std::vector<Eigen::Vector3d> gtPts;
		
		/*
		parseCalibFile( intrinsicsFile, K );		
		parseCameraFile( camFile, cameras );
		parsePtsFile( pointFile, points3d );		
		loadGroundTruth( camGT, pointGT, gtRots, gtTrans, gtPts );
		
		
		//printLoadedData( K, cameras, points3d );
		
		//testOpenCVImplementation( K, cameras, points3d );
		testMVGImplementation( K, cameras, points3d, gtRots, gtTrans, gtPts );
		 */
		testSimulation();
	}
	catch (const cvt::Exception & e) {
		std::cout << e.what() << std::endl;
	}	
	
	return 0;
}
