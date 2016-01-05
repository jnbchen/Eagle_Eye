#include "Localizer.h"


Localizer::Localizer(const DerWeg::ConfigReader& cfg, const CalibIO& calibration):
    m_oStateVector(5,1, CV_32FC1),
    m_oCovarianceMatrix(5,5, CV_32FC1),
    m_oStateTransitionMatrix(5,5, CV_32FC1),
    m_oProcessNoise(5,5, CV_32FC1),
    m_oCameraIntrinsics(3,3, CV_32FC1),
    m_oMeasModel(3,5,CV_32FC1)
{
    m_bLocalizerInit = false;

    LOUT("Init localizer" << std::endl);
    std::vector<std::string> ids;
    if (!cfg.get ("CameraLocalization::landmarks", ids)) {
        throw std::invalid_argument ("readLandmarks: key CameraLocalization::landmarks not found");
    }
    for (unsigned int i=0; i<ids.size(); i++) {
        LOUT("Reading LM #" << i << std::endl);
        std::vector<std::string> params;
        if (!cfg.get ((std::string("CameraLocalization::L")+ids[i]).c_str(), params)) {
            throw std::invalid_argument (std::string("readLandmarks: key L")+ids[i]+std::string(" not found"));
        }
        if (params.size()<4) {
            throw std::invalid_argument (std::string("readLandmarks: key L")+ids[i]+std::string(" incomplete"));
        }
        LandmarkPosition lm;
        std::stringstream inout;
        inout << params[0] << ' ' << params[1] << ' ' << params[2] << ' ' << params[3]; // << ' ' << params[4] << std::flush;
        inout >> lm.ID >> lm.H >> lm.X >> lm.Y >> lm.Theta;
        m_asLandmarkLUT.push_back (lm);
    }

    /// read camera paramters for the wide angle camera
    LOUT("Init calib" << std::endl);
    cv::Rect myROI(0, 0, 3, 3);
    m_oCameraIntrinsics = cv::Mat(calibration.cameras.at(0).P_rect,true);
    m_oCameraIntrinsics = m_oCameraIntrinsics(myROI);   // Calibration matrix is 3x4, so we cut it to 3x3
    LOUT("Done" << std::endl);

}

Localizer::~Localizer()
{

}

int Localizer::FindPosition(std::vector<Landmark> &i_voLandmarks, float fDeltaT)
{
    std::vector<cv::Mat> assumptions;
    std::vector< std::vector<int> > supporters;
    std::vector< double > errors;
    int i_winnerIndex= 0;
    double error_threshold = 100; ///@todo Wert finden/Parametrisieren

    //std::cout << "Entered find position" << std::endl;
    if (i_voLandmarks.size() > 1)
    {
        /// Step 1: For every combination of seen landmarks, calculate triangulation and
        ///         test this assumption on other landmarks by calculating reprojection error
        ///         those, that have little error, are kept as supporters
        for(int i = 0; i<i_voLandmarks.size(); i++)
        {
            for(int j = i+1; j<i_voLandmarks.size(); j++)
            {
                /// use triangulation to generate measurement
                LandmarkPosition temp;
                temp = GetLandmarkFromID(i_voLandmarks[i].nID);
                cv::Mat point1_world(3, 1, CV_32FC1);
                point1_world.at<float>(0,0) = temp.X;
                point1_world.at<float>(1,0) = temp.Y;
                point1_world.at<float>(2,0) = temp.H;
                temp = GetLandmarkFromID(i_voLandmarks[j].nID);
                cv::Mat point2_world(3, 1, CV_32FC1);
                point2_world.at<float>(0,0) = temp.X;
                point2_world.at<float>(1,0) = temp.Y;
                point2_world.at<float>(2,0) = temp.H;
                cv::Mat point1_image(2, 1, CV_32FC1);
                point1_image.at<float>(0,0) = i_voLandmarks[i].oPosition.x;
                point1_image.at<float>(1,0) = i_voLandmarks[i].oPosition.y;
                cv::Mat point2_image(2, 1, CV_32FC1);
                point2_image.at<float>(0,0) = i_voLandmarks[j].oPosition.x;
                point2_image.at<float>(1,0) = i_voLandmarks[j].oPosition.y;

                assumptions.push_back(TriangulateTwoPoints(point1_world,point1_image,point2_world,point2_image));

                //visualizeLandmarks(i_voLandmarks,assumptions.back());
                //cv::waitKey();
                std::vector<int> support;
                for(int k = 0; k<i_voLandmarks.size(); k++)
                {
                    cv::Mat errorMatrix = calcReprojectionError(i_voLandmarks[k], assumptions.back());
                    double error = cv::norm(errorMatrix);
                    //std::cout << "Error is " << error << std::endl;

                    if (error < error_threshold){
                        support.push_back(k);
                        errors.push_back(error);
                    }
                }
                supporters.push_back(support);

            }
        }
        /// Step 2: Choose the assumption with the most supporters
        int maxSupporters = 0;
        for(int i = 0; i<supporters.size(); i++)
        {
            if (maxSupporters < supporters[i].size())
            {
                maxSupporters = supporters[i].size();
                i_winnerIndex = i;
            }
        }
        if (maxSupporters ==0)
        {
            //std::cout << "Did not find supporters" << std::endl;
            return 1; ///@todo Make warning
        }
    }
    else if (i_voLandmarks.size() == 1)    // Only one landmark
    {
        //std::cout << "Using only one landmark!" << std::endl;
        /// Step 1:
        std::vector<cv::Mat> id_points;
        std::vector<cv::Mat> corner_points;
        GetPointsFromID(i_voLandmarks[0].nID,corner_points,id_points);
        for(int i = 0; i<corner_points.size(); i++)
        {
            for(int j = i; j<corner_points.size(); j++)
            {
                if(j != i)
                { /// only if the two points are not the same
                    /// use triangulation to generate measurement
                    cv::Mat point1_world = corner_points[i];
                    cv::Mat point2_world = corner_points[j];
                    cv::Mat point1_image(2, 1, CV_32FC1);
                    point1_image.at<float>(0,0) = i_voLandmarks[0].voCorners[i].x;
                    point1_image.at<float>(1,0) = i_voLandmarks[0].voCorners[i].y;
                    cv::Mat point2_image(2, 1, CV_32FC1);
                    point2_image.at<float>(0,0) = i_voLandmarks[0].voCorners[j].x;
                    point2_image.at<float>(1,0) = i_voLandmarks[0].voCorners[j].y;

                    assumptions.push_back(TriangulateTwoPoints(point1_world,point1_image,point2_world,point2_image));

                    cv::Mat errorMatrix = calcReprojectionError(i_voLandmarks[0], assumptions.back());
                    double error = cv::norm(errorMatrix);

                    if (error < error_threshold)
                    {
                        std::vector<int> support;
                        support.push_back(0);
                        errors.push_back(error);
                        supporters.push_back(support);
                    } else {
                        assumptions.pop_back(); // Remove the assumption again
                    }
                }
            }
        }
        /// Step 2: Choose the assumption with the smallest error
        double minError = DBL_MAX;
        for(int i = 0; i<errors.size(); i++)
        {
            if (minError > errors[i])
            {
                minError = errors[i];
                i_winnerIndex = i;
            }
        }
        if (minError == DBL_MAX)
        {
            //std::cout << "Err too large" << std::endl;
            return 1;
        }
    }
    else //No landmarks received
    {
        PredictKF(fDeltaT); // Predict only and then return
        //std::cout << "no landmarks found" << std::endl;
        return 1;
    }


    /// Step 3: Optimize this assumption locally, using the supporters!
//    std::cout << i_voLandmarks.size() << " Landmarks, " << assumptions.size() << " Assumptions, "<< std::endl;
//    double error = cv::norm(calcReprojectionError(i_voLandmarks.at(supporters[i_winnerIndex].at(0)),assumptions[i_winnerIndex]));
//    std::cout << "Error before: " << error << std::endl;
//    visualizeLandmarks(i_voLandmarks,m_oStateVector);
//    cv::waitKey(0);
    Eigen::Vector3d pose_guess;    // Take the winning assumption
    pose_guess(0) = assumptions[i_winnerIndex].at<float>(0,0);
    pose_guess(1) = assumptions[i_winnerIndex].at<float>(1,0);
    pose_guess(2) = assumptions[i_winnerIndex].at<float>(2,0);
    // LibMV initialization and minimization
    PoseFunctor a;
    a.pLandmarks = &i_voLandmarks;
    a.pSupporters = &supporters[i_winnerIndex];
    a.localizer = this;
    libmv::LevenbergMarquardt<PoseFunctor> optimizer(a);
    libmv::LevenbergMarquardt<PoseFunctor>::SolverParameters Param;
    optimizer.minimize(Param, &pose_guess);
    cv::Mat pose_measurement(3,1,CV_32FC1);
    pose_measurement.at<float>(0,0) = pose_guess(0);
    pose_measurement.at<float>(1,0) = pose_guess(1);
    pose_measurement.at<float>(2,0) = pose_guess(2);
    // Correct angle
    while(M_PI < pose_measurement.at<float>(2,0))
    {
        pose_measurement.at<float>(2,0) -= 2*M_PI;
    }
    while(-M_PI > pose_measurement.at<float>(2,0))
    {
        pose_measurement.at<float>(2,0) += 2*M_PI;
    }

    /// Step 4: Update Kalmanfilter
    if(!m_bLocalizerInit)
    {
        cv::Mat oCovariance = 1000*cv::Mat::eye(5,5,CV_32FC1);
        oCovariance.at<float>(2,2) = 100;
        oCovariance.at<float>(4,4) = 100;
        InitKF(pose_measurement, oCovariance);
        return 0;
    }
    PredictKF(fDeltaT);

//    cv::Mat oMeasNoise = cv::Mat::zeros(3,3,CV_32FC1);
//    for(int k = 0; k<supporters[i_winnerIndex].size(); k++)
//    {
//        oMeasNoise += calcReprojectionError(i_voLandmarks[supporters[i_winnerIndex].at(k)], assumptions.back());
//    }
//    oMeasNoise /= supporters[i_winnerIndex].size();
    /// noise is just a guess
    cv::Mat oMeasNoise = 1000*cv::Mat::eye(3,3,CV_32FC1);
    oMeasNoise.at<float>(2,2) = (1*M_PI/180)*(1*M_PI/180);
    UpdateKF(pose_measurement, oMeasNoise);

    /// DEBUG: Use measurement without filter
//    m_oStateVector.at<float>(0,0) = pose_measurement.at<float>(0,0);
//    m_oStateVector.at<float>(1,0) = pose_measurement.at<float>(1,0);
//    m_oStateVector.at<float>(2,0) = pose_measurement.at<float>(2,0);

//    error = cv::norm(calcReprojectionError(i_voLandmarks.at(supporters[i_winnerIndex].at(0)),m_oStateVector));
//    std::cout << "Error after: " << error << std::endl;
//    visualizeLandmarks(i_voLandmarks,m_oStateVector);
//    cv::waitKey(0);
    return 0;
}

/////////////////////////////////////
/// KALMAN FILTER
/////////////////////////////////////
int Localizer::InitKF(cv::Mat& i_oInitialState, cv::Mat& i_oInitialCovariance)
{
    /// set initial state vector
    m_oStateVector = cv::Mat::zeros(5,1,CV_32FC1);
    m_oStateVector.at<float>(0,0) = i_oInitialState.at<float>(0,0);
    m_oStateVector.at<float>(1,0) = i_oInitialState.at<float>(1,0);
    m_oStateVector.at<float>(2,0) = i_oInitialState.at<float>(2,0);
    /// set initial covariance
    m_oCovarianceMatrix = i_oInitialCovariance;
    /// set process noise matrix
    m_oProcessNoise = cv::Mat::eye(5,5,CV_32FC1);
    m_oProcessNoise.at<float>(0,0) = 10;                            // ca. 3mm
    m_oProcessNoise.at<float>(1,1) = 10;                            // ca. 3mm
    m_oProcessNoise.at<float>(2,2) = (5*M_PI/180)*(5*M_PI/180);     // 5°
    m_oProcessNoise.at<float>(3,3) = 1000;                          // ca. 31 mm/s
    m_oProcessNoise.at<float>(4,4) = (15*M_PI/180)*(15*M_PI/180);   // 15°/s
    /// set measurement model matrix
    m_oMeasModel = cv::Mat::zeros(3,5,CV_32FC1);
    m_oMeasModel.at<float>(0,0) = 1;
    m_oMeasModel.at<float>(1,1) = 1;
    m_oMeasModel.at<float>(2,2) = 1;

    m_bLocalizerInit = true;
    return 0;
}

int Localizer::PredictKF(float fDeltaT)
{
    float fTheta    = m_oStateVector.at<float>(2,0);
    float fV        = m_oStateVector.at<float>(3,0);
    float fThetaDot = m_oStateVector.at<float>(4,0);

    /// predict state
    /// Kalman Update: x^  = A*x + B*u (here without input u)
    /* Transistion matrix:
     *      X   Y   Theta   v               Theta_dot
     * X    1   0   0       dt*cos(Theta)   0
     * Y    0   1   0       dt*sin(Theta)   0
     * Th   0   0   1       0               dt
     * v    0   0   0       1               0
     * Th_d 0   0   0       0               1
     */
    m_oStateTransitionMatrix = cv::Mat::eye(5,5,CV_32F);    // Constant velocity, constant yawrate
    m_oStateTransitionMatrix.at<float>(0, 3) = fDeltaT*cos(fTheta);
    m_oStateTransitionMatrix.at<float>(1, 3) = fDeltaT*sin(fTheta);
    m_oStateTransitionMatrix.at<float>(2, 4) = fDeltaT;
    m_oStateVector = m_oStateTransitionMatrix*m_oStateVector;

//    /// predict state with constant circular velocity model
//    if (fThetaDot == 0){
//        m_oStateVector.at<float>(0,0) += fV*fDeltaT*cos(fTheta);
//        m_oStateVector.at<float>(1,0) += fV*fDeltaT*sin(fTheta);
//    } else {
//        m_oStateVector.at<float>(0,0) += fV/fThetaDot*(-sin(fTheta)+sin(fTheta+fThetaDot*fDeltaT));
//        m_oStateVector.at<float>(1,0) += fV/fThetaDot*(+cos(fTheta)-sin(fTheta+fThetaDot*fDeltaT));
//    }
//    m_oStateVector.at<float>(2,0) += fThetaDot*fDeltaT;
//    m_oStateVector.at<float>(3,0) = m_oStateVector.at<float>(3,0);
//    m_oStateVector.at<float>(4,0) = m_oStateVector.at<float>(4,0);

    /// Ensure orientation is within +-pi
    while(M_PI < m_oStateVector.at<float>(2,0))
    {
        m_oStateVector.at<float>(2,0) -= 2*M_PI;
    }
    while(-M_PI > m_oStateVector.at<float>(2,0))
    {
        m_oStateVector.at<float>(2,0) += 2*M_PI;
    }


    //! Compute noise process noise covariance Q
    cv::Mat oW = cv::Mat::eye(5,5,CV_32F);
    oW.at<float>(0, 2) = -fDeltaT*fV*sin(fTheta);
    oW.at<float>(0, 3) = fDeltaT*cos(fTheta);
    oW.at<float>(1, 2) = fDeltaT*fV*cos(fTheta);
    oW.at<float>(1, 3) = fDeltaT*sin(fTheta);
    oW.at<float>(2, 4) = fDeltaT;

//    /// set process noise matrix
//    cv::Mat temp = cv::Mat::eye(5,5,CV_32FC1);
//    temp.at<float>(0,0) = fDeltaT*fV*(0.15*cos(fTheta)+0.05*sin(fTheta));
//    temp.at<float>(1,1) = fDeltaT*fV*(0.15*sin(fTheta)+0.05*cos(fTheta));
//    temp.at<float>(2,2) = (5*M_PI/180)*(5*M_PI/180);
//    temp.at<float>(3,3) = fDeltaT*100;
//    temp.at<float>(4,4) = (5*M_PI/180)*(5*M_PI/180);

    /// predict covariance
    /// Kalman Update: P^  = A*P*A' + Q
    m_oCovarianceMatrix = m_oStateTransitionMatrix*m_oCovarianceMatrix*m_oStateTransitionMatrix.t() + oW*m_oProcessNoise*oW.t(); // temp; //

    return 0;
}

int Localizer::UpdateKF(cv::Mat& i_oMeasurement, cv::Mat& i_oMeasurementNoise)
{
    ///Discrete Kalman filter measurement update
    /* Measurement is of the form
     *      (X)
     * z =  (Y)
     *      (Z)
     * The measurement Model H (m_oMeasModel) is of the form
     *      (   1   0   0   0   0)
     * H =  (   0   1   0   0   0)
     *      (   0   0   1   0   0)
     * since the measurement and the first three elements of state vector are the same.
     * The measurement noise is of the form:
     *      ( sigma_x   0       0           )
     * R =  ( 0         sigma_y 0           )
     *      ( 0         0       sigma_theta )
     * */
    /// compute kalman matrix
    /// K = P^*H'*(H*P^*H' + R)^-1
    cv::Mat oTemp   = m_oCovarianceMatrix*m_oMeasModel.t();
    cv::Mat oS      = m_oMeasModel*oTemp + i_oMeasurementNoise;
    cv::Mat oK      = oTemp*oS.inv();

    /// update state vector
    /// x = x^ + K*(z-H*x^)
    cv::Mat oInnovation = i_oMeasurement - m_oMeasModel*m_oStateVector;
    /// Ensure orientation is within +-pi
    while(M_PI < oInnovation.at<float>(2,0))
    {
        oInnovation.at<float>(2,0) -= 2*M_PI;
    }
    while(-M_PI > oInnovation.at<float>(2,0))
    {
        oInnovation.at<float>(2,0) += 2*M_PI;
    }
    m_oStateVector = m_oStateVector + oK*oInnovation;

    /// Ensure orientation is within +-pi
    while(M_PI < m_oStateVector.at<float>(2,0))
    {
        m_oStateVector.at<float>(2,0) -= 2*M_PI;
    }
    while(-M_PI > m_oStateVector.at<float>(2,0))
    {
        m_oStateVector.at<float>(2,0) += 2*M_PI;
    }

    /// update covariance
    /// P = (I - K*H)*P^
    /// todo: get size and type of eye matrix from inputs
    m_oCovarianceMatrix = (cv::Mat::eye(5,5, CV_32FC1) - oK*m_oMeasModel)*m_oCovarianceMatrix;

    return 0;
}

/////////////////////////////////////
/// HELPER FUNCTIONS
/////////////////////////////////////

LandmarkPosition Localizer::GetLandmarkFromID(int ID)
{
    for(unsigned int i = 0; i < m_asLandmarkLUT.size(); i++)
    {
        if(ID == m_asLandmarkLUT[i].ID)
        {
            return m_asLandmarkLUT[i];
            break; /// break because there can be only one landmark per ID
        }
    }
    return LandmarkPosition();
}

void Localizer::GetPointsFromID(int ID, std::vector<cv::Mat> &corner_points, std::vector<cv::Mat> &id_points)
{
    ///--------------------------------------------------------------------------------------///
    /// ID of a landmark is coded see http://hagisonic.com/ for information on pattern
    ///--------------------------------------------------------------------------------------///

    /// Returns a vector of all landmark points in world coordinates
    /* Landmark IDs are coded:
     * the binary values ar coded:  x steps are binary shifts within 4 bit blocks
     *                              y steps are binary shifts of 4 bit blocks
     *
     *  Since we are looking at the underside of the landmarks, while at the same time defining the world coordinate frame in birdsview,
     *  the coordinate systems are switched. It looks like this:
     *  WORLD ---> y
     *  |
     *  |       3   .   .   . y
     *  V       .   .   .   . ^
     *  x       .   .   .   . |
     *          2   .   .   1 |
     *          x <--- LANDMARK
     * Therefor following conversion is performed:
     * x_world = -y_landmark
     * y_world = -x_landmark
     */
    //std::cout << "Got ID: " << ID << std::endl;
    double distance_between_points = 80; //80mm = 8cm
    LandmarkPosition lm_pos = GetLandmarkFromID(ID);
    cv::Mat tmp_point(4,1,CV_32FC1);
    tmp_point.at<float>(3,0) = 1.;
    int x_world, y_world, x_landmark, y_landmark;

    /// Add corner points @todo what if we have a 3*3 landmark?
    /// The landmark position is the middle between the two outer corners, so it is located at x=3.0/2.0 and y = 3.0/2.0
    /// Those coordinates are subtracted from each point, then they are rotated and translated according to the landmark position
    /// Corner 1
    x_landmark = 0;y_landmark = 0;
    x_world = -y_landmark;y_world = -x_landmark;
    tmp_point.at<float>(0,0) = lm_pos.X + distance_between_points*( (x_world+3.0/2.0)*cos(lm_pos.Theta) + (y_world+3.0/2.0)*-sin(lm_pos.Theta) ); /// x-axis points down, y-axis to the right (see pdf)
    tmp_point.at<float>(1,0) = lm_pos.Y + distance_between_points*( (x_world+3.0/2.0)*sin(lm_pos.Theta) + (y_world+3.0/2.0)*cos(lm_pos.Theta) );
    tmp_point.at<float>(2,0) = lm_pos.H;
    corner_points.push_back(tmp_point.clone());
    /// Corner 2
    x_landmark = 3;y_landmark = 0;
    x_world = -y_landmark;y_world = -x_landmark;
    tmp_point.at<float>(0,0) = lm_pos.X + distance_between_points*( (x_world+3.0/2.0)*cos(lm_pos.Theta) + (y_world+3.0/2.0)*-sin(lm_pos.Theta) ); /// x-axis points down, y-axis to the right (see pdf)
    tmp_point.at<float>(1,0) = lm_pos.Y + distance_between_points*( (x_world+3.0/2.0)*sin(lm_pos.Theta) + (y_world+3.0/2.0)*cos(lm_pos.Theta) );
    tmp_point.at<float>(2,0) = lm_pos.H;
    corner_points.push_back(tmp_point.clone());
    /// Corner 3
    x_landmark = 3;y_landmark = 3;
    x_world = -y_landmark;y_world = -x_landmark;
    tmp_point.at<float>(0,0) = lm_pos.X + distance_between_points*( (x_world+3.0/2.0)*cos(lm_pos.Theta) + (y_world+3.0/2.0)*-sin(lm_pos.Theta) ); /// x-axis points down, y-axis to the right (see pdf)
    tmp_point.at<float>(1,0) = lm_pos.Y + distance_between_points*( (x_world+3.0/2.0)*sin(lm_pos.Theta) + (y_world+3.0/2.0)*cos(lm_pos.Theta) );
    tmp_point.at<float>(2,0) = lm_pos.H;
    corner_points.push_back(tmp_point.clone());

    /// Find and add ID points
    int col = 0;
    for (int y=0;y<4;y++)   // For every column
    {
        col = (ID % int_pow(16,y+1));   // Modulo 16^(i+1) tells us how much this row contributed to the ID
        col /= int_pow(16,y);
        ID -= col;
        // Convert to binary
        for (int x=0;x<4;x++) { // For every row
            if (col%2 != 0) {   // Modulo 2 effectively converts the number to binary. If this returns 1, we have a point
                // Point found
                x_landmark = x;y_landmark = y;
                x_world = -y_landmark;y_world = -x_landmark;
                tmp_point.at<float>(0,0) = lm_pos.X + distance_between_points*( (x_world+3.0/2.0)*cos(lm_pos.Theta) + (y_world+3.0/2.0)*-sin(lm_pos.Theta) );
                tmp_point.at<float>(1,0) = lm_pos.Y + distance_between_points*( (x_world+3.0/2.0)*sin(lm_pos.Theta) + (y_world+3.0/2.0)*cos(lm_pos.Theta) );
                tmp_point.at<float>(2,0) = lm_pos.H;
                //                std::cout << "id_points: " << tmp_p.at<float>(0,0) << " " << tmp_p.at<float>(1,0) << " "<< tmp_p.at<float>(2,0) << " "<< std::endl;
                //                std::cout << "x: " << x << " y: " << y << " "<< std::endl;
                id_points.push_back(tmp_point.clone());
            }
            col /= 2;
        }
    }
}

cv::Mat Localizer::calcReprojectionError(Landmark& lm, cv::Mat position)
{

    /// Step 1: Get individual points of landmark in world coordinates
    std::vector<cv::Mat> id_points;
    std::vector<cv::Mat> corner_points;
    GetPointsFromID(lm.nID,corner_points,id_points);
    float theta = position.at<float>(2,0);

    if (id_points.size() != lm.voIDPoints.size() || corner_points.size() != lm.voCorners.size()){
        std::cout << "WARNING; THE AMOUNT OF POINTS MISMATCHES!";
        std::cout << "Landmark points: " << id_points.size() << "\t Seen points: " << lm.voIDPoints.size() << std::endl;
        std::cout << "Landmark corners: " << corner_points.size() << "\t Seen corners: " << lm.voCorners.size() << std::endl;
    }


    cv::Mat errorMatrix = cv::Mat(2*(lm.voIDPoints.size() + lm.voCorners.size()),1,CV_32FC1);

    /// Step 2: Create matrices for world->camera transformation

    cv::Mat oTrafo = cv::Mat::eye(4,4,CV_32FC1);
    oTrafo.at<float>(0,0) = cos(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
    oTrafo.at<float>(0,1) = -sin(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
    oTrafo.at<float>(1,0) = sin(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
    oTrafo.at<float>(1,1) = cos(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
    oTrafo.at<float>(0,3) =  position.at<float>(0,0);
    oTrafo.at<float>(1,3) =  position.at<float>(1,0);

    cv::Mat oTrafoInv = oTrafo.inv();
    //cv::Mat oCamInv = m_oCameraIntrinsics.inv();
    int nErrorIndex = 0;



    for(int nI = 0; nI < id_points.size(); nI++)
    {
        id_points[nI] = oTrafoInv*id_points[nI];
        id_points[nI] = cv::Mat(id_points[nI], cv::Rect(0,0,1,3));
        id_points[nI] = m_oCameraIntrinsics*id_points[nI];
        id_points[nI] *= 1/id_points[nI].at<float>(2,0);

        errorMatrix.at<float>(nErrorIndex++,0) = ( id_points[nI].at<float>(0,0)-lm.voIDPoints[nI].x );
        errorMatrix.at<float>(nErrorIndex++,0) = ( id_points[nI].at<float>(1,0)-lm.voIDPoints[nI].y );
    }

    for(int nI = 0; nI < corner_points.size(); nI++)
    {
        corner_points[nI] = oTrafoInv*corner_points[nI];
        corner_points[nI] = cv::Mat(corner_points[nI], cv::Rect(0,0,1,3));
        corner_points[nI] = m_oCameraIntrinsics*corner_points[nI];
        corner_points[nI] *= 1/corner_points[nI].at<float>(2,0);

        errorMatrix.at<float>(nErrorIndex++,0) = ( corner_points[nI].at<float>(0,0)-lm.voCorners[nI].x );
        errorMatrix.at<float>(nErrorIndex++,0) = ( corner_points[nI].at<float>(1,0)-lm.voCorners[nI].y );
    }

    return errorMatrix;

//    cv::Mat oRotation = cv::Mat::eye(3,3,CV_32FC1);
//    oRotation.at<float>(0,0) = cos(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
//    oRotation.at<float>(0,1) = sin(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
//    oRotation.at<float>(1,0) = -sin(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
//    oRotation.at<float>(1,1) = cos(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
//    cv::Mat oTranslation(3,1,CV_32FC1);
//    oTranslation.at<float>(0,0) = position.at<float>(0,0);
//    oTranslation.at<float>(1,0) = position.at<float>(1,0);
//    oTranslation.at<float>(2,0) = 0;

//    /// ID Points
//    for(int i=0; i<id_points.size(); i++)
//    {
//        /// Step 3: Convert every point into camera frame
//        float z = id_points[i].at<float>(2,0);
//        cv::Mat pt1_w = id_points[i];
//        cv::Mat pt2_i(3,1,CV_32FC1);
//        pt2_i.at<float>(0,0) = lm.voIDPoints[i].x;
//        pt2_i.at<float>(1,0) = lm.voIDPoints[i].y;
//        pt2_i.at<float>(2,0) = 1;
//        cv::Mat pt2_w = oRotation.inv()*m_oCameraIntrinsics.inv()*z*pt2_i + oTranslation;
//        id_points[i] = m_oCameraIntrinsics*oRotation*(id_points[i] - oTranslation);
//        id_points[i] *= 1/id_points[i].at<float>(2,0);

//        /// Step 4: Add up error
////        cv::Mat error_w = pt2_w-pt1_w;
////        errorMatrix.at<float>(0,0) += pow(error_w.at<float>(0,0),2);
////        errorMatrix.at<float>(1,1) += pow(error_w.at<float>(1,0),2);
//        errorMatrix.at<float>(0,0) += fabs( id_points[i].at<float>(0,0)-lm.voIDPoints[i].x );
//        errorMatrix.at<float>(1,1) += fabs( id_points[i].at<float>(1,0)-lm.voIDPoints[i].y );
//    }

//    /// Corner Points
//    for(int i=0; i<corner_points.size(); i++)
//    {
//        /// Step 3: Convert every point into camera frame
//        float z = corner_points[i].at<float>(2,0);
//        cv::Mat pt1_w = corner_points[i];
//        cv::Mat pt2_i(3,1,CV_32FC1);
//        pt2_i.at<float>(0,0) = lm.voCorners[i].x;
//        pt2_i.at<float>(1,0) = lm.voCorners[i].y;
//        pt2_i.at<float>(2,0) = 1;
//        cv::Mat pt2_w = oRotation.inv()*m_oCameraIntrinsics.inv()*z*pt2_i + oTranslation;
//        corner_points[i] = m_oCameraIntrinsics*oRotation*(corner_points[i] - oTranslation);
//        corner_points[i] *= 1/corner_points[i].at<float>(2,0);

//        /// Step 4: Add up error
////        cv::Mat error_w = pt2_w-pt1_w;
////        errorMatrix.at<float>(0,0) += pow(error_w.at<float>(0,0),2);
////        errorMatrix.at<float>(1,1) += pow(error_w.at<float>(1,0),2);
//        errorMatrix.at<float>(0,0) += fabs( corner_points[i].at<float>(0,0)-lm.voCorners[i].x );
//        errorMatrix.at<float>(1,1) += fabs( corner_points[i].at<float>(1,0)-lm.voCorners[i].y );
//    }

//    /// Mean the errors
//    errorMatrix /= (corner_points.size() + id_points.size());
//    /// Calc Angle error
//    double ang1 = atan2(corner_points[2].at<float>(1,0)-corner_points[0].at<float>(1,0) , corner_points[2].at<float>(0,0)-corner_points[0].at<float>(0,0));
//    double ang2 = atan2(lm.voCorners[2].y - lm.voCorners[0].y , lm.voCorners[2].x - lm.voCorners[0].x);
//    ang1 = std::fmod(ang1,2*M_PI);
//    ang2 = std::fmod(ang2,2*M_PI);
//    double angErr = fabs(ang1-ang2);
//    if (angErr > M_PI) angErr = 2*M_PI - angErr;
//    errorMatrix.at<float>(2,2) = angErr;
//    return errorMatrix;
}

cv::Mat Localizer::points2mat(int count,...)
{
        va_list ap;
        int j;
        cv::Mat output(count,1,CV_32FC1);
        va_start(ap, count); /* Requires the last fixed parameter (to get the address) */
        for (j = 0; j < count; j++) {
            output.at<float>(j,0) = va_arg(ap, double); /* Increments ap to the next argument. */
        }
        va_end(ap);
        return output;
}

cv::Mat Localizer::TriangulateTwoPoints(cv::Mat& i_pointOneWorld,cv::Mat& i_pointOneImg,cv::Mat& i_pointTwoWorld,cv::Mat& i_pointTwoImg)
{
    ///    Pos = Vehicle Position
    ///    a1  = angle from landmark 1 to vehicle
    ///    b1  = viewing angle of landmark 1
    ///    R1  = radius around landmark 1 determined by elevation angle
    ///    L1  = Landmark position in global coordinate frame
    ///
    ///     [PosX]        [L1X]       [cos(a1)    -sin(a1)]   [R1]
    /// (1) [    ]    =   [   ]   +   [                   ]*  [  ]
    ///     [PosY]        [L1Y]       [sin(a1)     cos(a1)]   [ 0]
    ///
    ///     the same holds for landmark 2
    ///
    ///     [PosX]        [L2X]       [cos(a2)    -sin(a2)]   [R2]
    /// (2) [    ]    =   [   ]   +   [                   ]*  [  ]
    ///     [PosY]        [L2Y]       [sin(a2)     cos(a2)]   [ 0]
    ///
    ///     furthermore:
    ///
    ///
    /// (3) Theta = a1 - 180 - b1 = a2 - 180 - b2
    ///
    ///     This leads to: a1 = a2 - b2 + b1
    ///
    ///     Setting (1) equal to (2) and substituting a1 by (3):
    ///     [L1X - L2X]     (   [1  0]      [ cos(b2-b1)  sin(b2-b1)]) [cos(a2)]
    /// (4) [         ] =   (R2*[    ] - R1*[                       ])*[       ]
    ///     [L1X - L2X]     (   [0  1]      [-sin(b2-b1)  cos(b2-b1)]) [sin(a2)]



    /// Landmark positions in global coordinate frame
    cv::Mat oL1(2, 1, CV_32FC1);
    cv::Mat oL2(2, 1, CV_32FC1);
    double fH1 =1, fH2 =1;

    /// match landmarks from LUT with observations and get their positions
    /// Landmark 1
    oL1.at<float>(0,0)  = i_pointOneWorld.at<float>(0,0);
    oL1.at<float>(1,0)  = i_pointOneWorld.at<float>(1,0);
    fH1                 = i_pointOneWorld.at<float>(2,0);
    /// Landmark 2
    oL2.at<float>(0,0)  = i_pointTwoWorld.at<float>(0,0);
    oL2.at<float>(1,0)  = i_pointTwoWorld.at<float>(1,0);
    fH2                 = i_pointTwoWorld.at<float>(2,0);

    /// intrinsics and stuff
    float fF = m_oCameraIntrinsics.at<float>(0,0); /// focal length in pixels
    float fCx = m_oCameraIntrinsics.at<float>(0,2);
    float fCy = m_oCameraIntrinsics.at<float>(1,2);

    /// this is to find the observation angles
    float fX = float(i_pointOneImg.at<float>(0,0) - fCx);
    float fY = float(i_pointOneImg.at<float>(1,0) - fCy);
    float fLength = sqrt(fX*fX + fY*fY);

    /// elevation angle of landmark 1
    float fTanElev = fLength/fF;
    /// radius of the search circle for landmark 1
    float fR1 = fH1*fTanElev;
    /// viewing angle for landmark 1
    float fBeta1 = atan2f(fY, fX);

    /// same for landmark 2
    fX = float(i_pointTwoImg.at<float>(0,0) - fCx);
    fY = float(i_pointTwoImg.at<float>(1,0) - fCy);
    fLength = sqrt(fX*fX + fY*fY);

    /// elevation angle of landmark 2
    fTanElev = fLength/fF;
    /// radius of the search circle for landmark 2
    float fR2 = fH2*fTanElev;
    /// viewing angle for landmark 2
    float fBeta2 = atan2f(fY, fX);


    /// matrices
    cv::Mat oRot1(2, 2, CV_32FC1);
    cv::Mat oRot2(2, 2, CV_32FC1);
    cv::Mat oTot(2,2, CV_32FC1);

    /// see derivation above for further information
    oRot1.at<float>(0,0) = cos(fBeta2 - fBeta1);
    oRot1.at<float>(0,1) = sin(fBeta2 - fBeta1);
    oRot1.at<float>(1,0) = -sin(fBeta2 - fBeta1);
    oRot1.at<float>(1,1) = cos(fBeta2 - fBeta1);

    oRot2 = cv::Mat::eye(2,2, CV_32FC1);

    oTot = fR2*oRot2 - fR1*oRot1;

    /// solution to (4)
    cv::Mat oDir2 = oTot.inv()*(oL1-oL2);

    /// position according to (2)
    cv::Mat oPos = oL2 + fR2*oDir2;

    /// orientation according to (3)
    float fOrientation = atan2f(oDir2.at<float>(1,0), oDir2.at<float>(0,0)) - M_PI - fBeta2;

    /// limit orientation angle between [-pi, pi]
    while(M_PI < fOrientation)
    {
        fOrientation -= 2*M_PI;
    }
    while(-M_PI > fOrientation)
    {
        fOrientation += 2*M_PI;
    }

    /// generate output state vector
    cv::Mat oOutput(3, 1, CV_32FC1);

    oOutput.at<float>(0,0) = oPos.at<float>(0,0);
    oOutput.at<float>(1,0) = oPos.at<float>(1,0);
    oOutput.at<float>(2,0) = fOrientation - M_PI/2;

    return oOutput;
}

void Localizer::visualizeLandmarks(std::vector<Landmark> &i_voLandmarks, cv::Mat position)
{
    std::stringstream out1;
    std::string txt;
    cv::Mat img = cv::Mat::zeros(1995,2600,CV_8UC3); ///@todo read those in from somewhere!
    cv::Point Test;
    double max_error= 0;
    for (int j = 0; j < i_voLandmarks.size(); j++)
    {
        /// Step 1: Get individual points of landmark in world coordinates
        std::vector<cv::Mat> id_points;
        std::vector<cv::Mat> corner_points;
        GetPointsFromID(i_voLandmarks[j].nID,corner_points,id_points);
        float theta = position.at<float>(2,0);

        /// Step 2: Create matrices for world->camera transformation
        cv::Mat oTrafo = cv::Mat::eye(4,4,CV_32FC1);
        oTrafo.at<float>(0,0) = cos(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
        oTrafo.at<float>(0,1) = -sin(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
        oTrafo.at<float>(1,0) = sin(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
        oTrafo.at<float>(1,1) = cos(theta + M_PI/2); //  + M_PI/2, because camera coordinate system is rotated to vehicle coordinate system
        oTrafo.at<float>(0,3) =  position.at<float>(0,0);
        oTrafo.at<float>(1,3) =  position.at<float>(1,0);

        cv::Mat oTrafoInv = oTrafo.inv();


        /// ID Points
        cv::Mat oErrVec = calcReprojectionError(i_voLandmarks[j],  position);
        double error = cv::norm(oErrVec);

        /// PLOT
        for(int i=0; i<id_points.size(); i++)
        {
            cv::Mat oTemp = oTrafoInv*(id_points[i]);
            oTemp = cv::Mat(oTemp, cv::Rect(0,0,1,3));
            oTemp = m_oCameraIntrinsics*oTemp;
            oTemp *= 1/oTemp.at<float>(2,0);

            Test = cvPoint(oTemp.at<float>(0,0),oTemp.at<float>(1,0));
            circle(img, Test, 3, cv::Scalar(0,0,255), 2);   //Red
            out1 << i;
            txt = out1.str();
            //            putText(img, txt, Test, 2, 0.4 ,cvScalar(0,0,255) );
            out1.str(std::string());
        }
        for(int i=0; i<corner_points.size(); i++)
        {
            cv::Mat oTemp = oTrafoInv*corner_points[i];
            oTemp = cv::Mat(oTemp, cv::Rect(0,0,1,3));
            oTemp = m_oCameraIntrinsics*oTemp;
            oTemp *= 1/oTemp.at<float>(2,0);


            Test = cvPoint(oTemp.at<float>(0,0),oTemp.at<float>(1,0));
            circle(img, Test, 3, cv::Scalar(255,0,255), 2); //Orange
            out1 << i;
            txt = out1.str();
            //putText(img, txt, Test, 2, 0.4 ,cvScalar(255,0,255) );
            out1.str(std::string());
        }
        for(int i=0; i<i_voLandmarks[j].voIDPoints.size(); i++)
        {
            circle(img, i_voLandmarks[j].voIDPoints[i], 2, cv::Scalar(255,0,0), 2);   //Blue
            out1 << i;
            txt = out1.str();
            //            putText(img, txt, i_voLandmarks[j].voIDPoints[i], 2, 0.4 ,cvScalar(255,0,0) );
            out1.str(std::string());
        }
        for(int i=0; i<i_voLandmarks[j].voCorners.size(); i++)
        {
            circle(img, i_voLandmarks[j].voCorners[i], 2, cv::Scalar(255,128,0), 2); //Magenta
            out1 << i;
            txt = out1.str();
            //putText(img, txt, i_voLandmarks[j].voCorners[i], 2, 0.4 ,cvScalar(255,128,0) );
            out1.str(std::string());
        }

        Test = cvPoint(i_voLandmarks[j].oPosition.x + 25,i_voLandmarks[j].oPosition.y + 25);
        out1 << "Error: " << error;
        txt = out1.str();
        putText(img, txt, Test, 2, 0.4 ,cvScalar(0,255,0) );
        out1.str(std::string());
        Test = cvPoint(i_voLandmarks[j].oPosition.x + 25,i_voLandmarks[j].oPosition.y - 25);
        out1 << "ID: " << i_voLandmarks[j].nID;
        txt = out1.str();
        putText(img, txt, Test, 2, 0.4 ,cvScalar(255,255,0) );
        out1.str(std::string());
        if (error>max_error) max_error = error;
    }   // Landmarks
//    Test = cvPoint(25,25);
//    out1 << "Max. Error: " << max_error;
//    txt = out1.str();
//    putText(img, txt, Test, 2, 1 ,cvScalar(0,0,255) );
//    out1.str(std::string());
    cv::namedWindow("Reprojection Image",CV_WINDOW_NORMAL);
    cv::imshow("Reprojection Image", img);
    cv::waitKey(1);
}

Eigen::VectorXd PoseFunctor::operator()(Eigen::Vector3d Input) const
{

    int nSize = 0;
    for(int nI = 0; nI < pSupporters->size(); nI++)
    {
        nSize += pLandmarks->at(pSupporters->at(nI)).voIDPoints.size();
        nSize += pLandmarks->at(pSupporters->at(nI)).voCorners.size();
    }
    Eigen::VectorXd Output(2*nSize);

    cv::Mat position(3, 1, CV_32FC1);
    //cv::eigen2cv(Input,position);
    while(M_PI < Input(2))
    {
        Input(2) -= 2*M_PI;
    }
    while(-M_PI > Input(2))
    {
        Input(2) += 2*M_PI;
    }
    position.at<float>(0,0) = Input(0);
    position.at<float>(1,0) = Input(1);
    position.at<float>(2,0) = Input(2);


    int nIndex = 0;
    for(int nI = 0; nI < pSupporters->size(); nI++)
    {
        cv::Mat errorMatrix = localizer->calcReprojectionError(pLandmarks->at(pSupporters->at(nI)),position);
        for(int nK = 0; nK < errorMatrix.rows; nK++)
        {
            Output(nIndex++) = errorMatrix.at<float>(nK, 0);
        }
    }

    return Output;
}


