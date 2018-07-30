#include "ekfslam.h"
using std::string;
using std::cout;
using std::endl;


ekfslam::ekfslam()
{
        //设定初始机器人坐标值
        odom_robotPosition(0) = 0;
	odom_robotPosition(1) = 0;
	odom_robotPosition(2) = 0;

	robotPosition = odom_robotPosition; 

        numLandmarks = 0;
	numObservedLandmarks = 0;
      
        sigma = Eigen::MatrixXd::Zero(3, 3); //初始协方差, 3x3 的单位矩阵       
        stateX = robotPosition; //机器人位姿和landmark的位置组成的状态,初始时只有机器人状态


        Rt << coV2, 0, 0,
              0, coV2, 0,
              0, 0, coW2;

        Qt << coV2, 0,
              0, coW2; 
      
/*        Rt << 0, 0, 0,
              0, 0, 0,
              0, 0, 0;

        Qt << 0, 0,
              0, 0; */
        

        scan_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "/scan", 1);
	odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom", 1);    
    
        sync = new  message_filters::Synchronizer<ekfslamSyncPolicy>(ekfslamSyncPolicy(5), *scan_sub, *odom_sub);

	current_time = ros::Time::now();
        last_time = ros::Time::now();

        sync->registerCallback(boost::bind(&ekfslam::combineCallback,this, _1, _2));

}


ekfslam::~ekfslam()
{



}


void ekfslam::combineCallback(const sensor_msgs::LaserScanConstPtr& scanMsgAddress, const nav_msgs::Odometry::ConstPtr& odomMsgAddress)  //回调中包含多个消息
{
          std::vector<double> laser_ranges;   //存储激光数据
          for(int i=0; i<NUM; i++)
          {
                laser_ranges.push_back(scanMsgAddress->ranges[i]);
                //cout<<laser_ranges[i]<<endl;
          }  

	  //int old_numLandmarks = numLandmarks;
      
          //extractLandmarksFromScan(laser_ranges);  //提取Landmark，若有新的landmark则加入landmarks
	  //update_sigmaAndstate(old_numLandmarks);  //若有新的landmark更新状态stateX和协方差矩阵sigma的维数
          

	  linear_speed = odomMsgAddress->twist.twist.linear.x;
          angular_speed = odomMsgAddress->twist.twist.angular.z;

          ekf(laser_ranges);


          
          cout<<numLandmarks<<endl;
          for(int i=0; i<numLandmarks; i++)
          {
               cout<<landmarks[i].rho<<"   "<<landmarks[i].theta<<endl;
               cout<<landmarks[i].consensusNum<<endl; ////局内点个数
          }
          cout<<endl;

          //cout<<stateX<<endl;
          //cout<<sigma<<endl;
          cout<<endl;
          

          displayPL(laser_ranges);  //rviz
          publishTF(odomMsgAddress);

	  
}





void ekfslam::ekf(std::vector<double> ranges)
{
   
            Eigen::VectorXd pre_stateX;
            Eigen::MatrixXd stateJacobian;

            //cout<<numLandmarks<<endl;
            cal_stateJacobian(pre_stateX, stateJacobian); //由机器人速度和角速度，求预测的状态和计算状态方程的Jacobian矩阵

            //cout<<pre_stateX<<endl;
            //cout<<stateJacobian<<endl;
           

             Eigen::Vector3d pre_robotPosition;
             pre_robotPosition(0) = pre_stateX(0);
             pre_robotPosition(1) = pre_stateX(1);
             pre_robotPosition(2) = pre_stateX(2);
                         

             Eigen::MatrixXd pre_sigma;
             pre_sigma = stateJacobian*sigma*stateJacobian.transpose(); //预测协方差矩阵(3+2*numLandmarks)X(3+2*numLandmarks)
            // pre_sigma(0, 0) = pre_sigma(0, 0)+coV2;
             //pre_sigma(1, 1) = pre_sigma(1, 1)+coV2;
             //pre_sigma(2, 2) = pre_sigma(2, 2)+coV2;
             //cout<<pre_sigma<<endl;
            

             //在机器人坐标系下提取的线         
             std::vector<Line> lineRs; 
             extractLineR(ranges, lineRs);  
                                                                   
             //根据观测到的landmarks迭代计算
             for(int j=0; j<numObservedLandmarks; j++)
             {
                    //由预估的位置预测的landmark位置（机器人坐标系下）与实际传感器观测的landmark位置（机器人坐标系下）相减，计算测量余差MeasurementError_j
                   //2X(3+2*numLandmarks)的矩阵
                   Eigen::MatrixXd outputJacobian; 
                   outputJacobian = cal_outputJacobian(observedLandmarks[j], observedLandmarks[j].id, pre_robotPosition);
                   
                   //cout<<outputJacobian<<endl<<endl;

                   Eigen::MatrixXd S;
                   S = outputJacobian*pre_sigma*outputJacobian.transpose() + Qt; //测量余差协方差2X2
                   
                   double Sd = S(0, 0)*S(1, 1) - S(0, 1)*S(1,0);  // 矩阵S行列式的值，用于判断S是否可逆
 //                  cout<<S<<endl<<endl;
                   
                   Eigen::MatrixXd K;

                   if(Sd!=0)
                   {
                       K = pre_sigma*outputJacobian.transpose()*S.inverse(); //kalman增益矩阵(3+2*numLandmarks)X2,要注意S矩阵不可逆的情况
                   }
                   else
                   {
                       K = Eigen::MatrixXd::Zero(3+2*numLandmarks, 2);        
                   }
                   //cout<<K<<endl<<endl<<endl;
                   

                   Eigen::Vector2d MeasurementError_j;
                   MeasurementError_j = MeasurementError(observedLandmarks[j], lineRs, pre_robotPosition);
 //                  cout<<MeasurementError_j<<endl;
/*
                      pre_stateX = pre_stateX + K*MeasurementError_j;  //更新预测state
                       //cout<<pre_stateX<<endl;

                       Eigen::MatrixXd I;
                       I = Eigen::MatrixXd::Identity(pre_sigma.rows(), pre_sigma.rows());

                       pre_sigma = (I-K*outputJacobian)*pre_sigma;
                       //cout<<pre_sigma<<endl<<endl<<endl;
                       
                        double lam = MeasurementError_j.transpose()*S*MeasurementError_j;                        
                        cout<<lam<<endl<<endl<<endl;
*/

                   double lam = MeasurementError_j.transpose()*S*MeasurementError_j;
                   if(lam<lambda)                  //数据关联中的参数，只有该足够小时才能进行更新计算
                   {
                       pre_stateX = pre_stateX + K*MeasurementError_j;  //更新预测state
                       //cout<<pre_stateX<<endl;

                       Eigen::MatrixXd I;
                       I = Eigen::MatrixXd::Identity(pre_sigma.rows(), pre_sigma.rows());

                       pre_sigma = (I-K*outputJacobian)*pre_sigma;
                       //cout<<pre_sigma<<endl<<endl<<endl;
                   }

             }
             
             sigma = pre_sigma;  //获得下一时刻的协方差矩阵
             stateX = pre_stateX;

             //实际的位置            
             robotPosition(0) = stateX(0);
             robotPosition(1) = stateX(1);
             robotPosition(2) = stateX(2);

             //cout<<numLandmarks<<endl;
             //cout<<stateX<<endl;
             //landmarks的状态也要更新
             std::vector<Landmark>::iterator iter;  
             for(iter = landmarks.begin(); iter != landmarks.end(); ++iter)  
             {  
                  int itid = (*iter).id;
                 (*iter).rho = stateX(2*itid+1);
                 (*iter).theta = stateX(2*itid+2);

                 (*iter).a = -1/tan( (*iter).theta ); 
                 (*iter).b = (*iter).rho/sin( (*iter).theta ); 

             }  
 /*            
             for(int it=0; it<numLandmarks; ++it)
             {
                  landmarks[it].rho = stateX(2*it+3);
                  landmarks[it].theta = stateX(2*it+4);
             }

*/ 

             int old_numLandmarks = numLandmarks; 
             observedLandmarks.clear();            
             extractLandmarksFromScan(ranges);  //提取Landmark，若有新的landmark则加入landmarks             
         
             update_sigmaAndstate(old_numLandmarks);                       
                          
}




//从传感器数据提取直线做为landmark,每个landmark都给一个特定的id
void ekfslam::extractLandmarksFromScan(std::vector<double> ranges)
{
        std::vector<Line> lines;
        extractLineW(ranges, lines);

        Landmark newLandmark;
        

        for(int i=0; i<lines.size(); i++)
        { 
               bool different = true;
              
               if(numLandmarks!=0)
               {
                    int j = 0;
                    while( (j<numLandmarks) && (different==true))
                    {
                            double rhoerror = fabs(lines[i].rho-landmarks[j].rho);
                            double thetaerror = fabs( pi_limit(lines[i].theta-landmarks[j].theta) ) ;    

                            if( (rhoerror < rhoThreshold) && (thetaerror < thetaThreshold) )  //判断是否与已经提取的landmark相同
                            {
                                  different = false;
                                  landmarks[j].totalTimesObserved += 1;

                                  observedLandmarks.push_back(landmarks[j]);  //添加到观测到的observedLandmark中
                            } 
                            j++;
                    }

                    
                   /* for(int j=0; j<numLandmarks; j++)
                    {
                            double rhoerror = fabs(lines[i].rho-landmarks[j].rho);
                            double thetaerror = fabs( pi_limit(lines[i].theta-landmarks[j].theta) ) ;    

                            if( (rhoerror < rhoThreshold*landmarks[j].rho) && (thetaerror < thetaThreshold) )  //判断是否与已经提取的landmark相同
                            {
                                  different = false;
                                  landmarks[j].totalTimesObserved += 1;

                                  observedLandmarks.push_back(landmarks[j]);  //添加到观测到的observedLandmark中
                                  break; //跳出本层循环
                            } 
                            else
                            {
                                  different = true;
                            }    
                    }  */         
               }      
               else
               {
                     different = true;
               }   

               if(different)
               {
                     newLandmark.rho = lines[i].rho;
                     newLandmark.theta = lines[i].theta;

                     newLandmark.a = lines[i].a;
                     newLandmark.b = lines[i].b;
                     
                     newLandmark.beginx = lines[i].beginx;
                     newLandmark.endx = lines[i].endx;

                     newLandmark.id = numLandmarks+1;
                     newLandmark.totalTimesObserved = 1;
                     newLandmark.consensusNum = lines[i].consensusNum;;

                     landmarks.push_back(newLandmark); 
                     observedLandmarks.push_back(newLandmark);

                     numLandmarks = landmarks.size();
                     numObservedLandmarks = observedLandmarks.size(); 
               }                        
                                   
        }
      
}





void ekfslam::cal_stateJacobian(Eigen::VectorXd& pre_stateX, Eigen::MatrixXd& stateJacobian) //计算状态方程的Jacobian矩阵
{
         current_time = ros::Time::now();
	 double dt = (current_time - last_time).toSec();
         //cout<<dt<<endl;
         double detaX;
         double detaY;
         //机器人的速度运动模型
         if(angular_speed == 0)
         {
	       detaX = linear_speed*cos( robotPosition(2) )*dt;
               detaY = linear_speed*sin( robotPosition(2) )*dt;
         }
         else
         {
               detaX = - (linear_speed/angular_speed)*sin(robotPosition(2)) + (linear_speed/angular_speed)*sin( robotPosition(2)+angular_speed*dt );
               detaY = (linear_speed/angular_speed)*cos(robotPosition(2)) - (linear_speed/angular_speed)*cos( robotPosition(2)+angular_speed*dt );
         }
         //cout<<detaX<<"  "<<detaY<<"   "<<linear_speed<<""<<angular_speed<<"   "<<dt<<endl;

	 pre_stateX = stateX;
         pre_stateX(0) = robotPosition(0) + detaX;
         pre_stateX(1) = robotPosition(1) + detaY;
         pre_stateX(2) = robotPosition(2) + angular_speed*dt;    //预测状态

     Eigen::Matrix3d G3; //计算机器人本身的Jacobian矩阵

     G3 << 1, 0, -detaY,
           0, 1, detaX,
           0, 0, 1;

     Eigen::MatrixXd Gxm(3, 2*numLandmarks);
     Gxm = Eigen::MatrixXd::Zero(3, 2*numLandmarks);

     Eigen::MatrixXd Gmx(2*numLandmarks, 3); 
     Gmx = Eigen::MatrixXd::Zero(2*numLandmarks, 3);

     Eigen::MatrixXd Gmm(2*numLandmarks, 2*numLandmarks);
     Gmm = Eigen::MatrixXd::Identity(2*numLandmarks, 2*numLandmarks);
     
     Eigen::MatrixXd sJacobian(3+2*numLandmarks, 3+2*numLandmarks);  //用分块矩阵一定要显示写出矩阵维数，不然会报错
     sJacobian << G3, Gxm,
                 Gmx, Gmm;

     stateJacobian = sJacobian;

     //cout<<pre_stateX<<endl;
     //cout<<stateJacobian<<endl;

     last_time = current_time;
}



//输出方程的jacobian矩阵
Eigen::MatrixXd ekfslam::cal_outputJacobian(Landmark observedLandmark, int observedLandmark_id, Eigen::Vector3d pre_robotPosition)
{

        //cout<<pre_robotPosition<<endl<<endl;
        int j = observedLandmark_id;

        Eigen::MatrixXd Hj(2,5);
        Hj = Eigen::MatrixXd::Zero(2,5); 
        
        double rhok = sqrt( pow(pre_robotPosition(0), 2)+pow(pre_robotPosition(1), 2) ); //pre_robotPosition(0)是预测的机器人坐标x的值
        double thetak;
        if(pre_robotPosition(0)!=0)
        {
               thetak = atan2(pre_robotPosition(1), pre_robotPosition(0));
        }
        else 
        {
               if( pre_robotPosition(1)>0 )
                    thetak = PI/2;
               else if( pre_robotPosition(1)<0 )
                    thetak = -PI/2;
               else
                    thetak = 0;
                   
        }

        double d = rhok*cos(observedLandmark.theta-thetak);

       //cout<<rhok<<"  "<<thetak<<"  "<<d<<endl<<endl;
       if(rhok!=0)
       {
             if(observedLandmark.rho>d) 
             {
                  Hj(0,0) = (1/rhok)*( pre_robotPosition(0)*cos(observedLandmark.theta-thetak) - pre_robotPosition(1)*sin(observedLandmark.theta-thetak) );
                  Hj(0,1) = (1/rhok)*( pre_robotPosition(1)*cos(observedLandmark.theta-thetak) + pre_robotPosition(0)*sin(observedLandmark.theta-thetak) );
                  Hj(0,3) = 1;
                  Hj(0,4) = sin(thetak);
                  Hj(1,2) = -1;
                  Hj(1,4) = 1;
            } 
            else
            {
                  Hj(0,0) = -(1/rhok)*( pre_robotPosition(0)*cos(observedLandmark.theta-thetak) - pre_robotPosition(1)*sin(observedLandmark.theta-thetak) );
                  Hj(0,1) = -(1/rhok)*( pre_robotPosition(1)*cos(observedLandmark.theta-thetak) + pre_robotPosition(0)*sin(observedLandmark.theta-thetak) );
                  Hj(0,3) = -1;
                  Hj(0,4) = -sin(thetak);
                  Hj(1,2) = -1;
                  Hj(1,4) = 1;
            }
        
       } 
        

        Eigen::MatrixXd Fj(5,3+2*numLandmarks);
        Fj = Eigen::MatrixXd::Zero(5,3+2*numLandmarks);        
        Fj(0,0) = 1;
        Fj(1,1) = 1;
        Fj(2,2) = 1;
        Fj(3,2*j+1) = 1;
        Fj(4,2*j+2) = 1;
        
        Eigen::MatrixXd outputJacobian_j(2, 3+2*numLandmarks);
        outputJacobian_j = Hj*Fj; //2X(3+2*numLandmarks)的矩阵

        //cout<<numLandmarks<<endl;
        //cout<<Hj<<endl;
       // cout<<Fj<<endl;
        //cout<<outputJacobian_j<<endl<<endl;
        return outputJacobian_j;
}




//计算测量余差(或叫新息)
Eigen::Vector2d ekfslam::MeasurementError(Landmark observedLandmark, std::vector<Line> lineRs, Eigen::Vector3d pre_robotPosition)
{                          
     //预测机器人坐标系下的observedLandmark的极坐标，世界坐标变换为机器人坐标
     Eigen::Vector2d pre_oLandmarkR;
     pre_oLandmarkR = WtoR(observedLandmark, pre_robotPosition); 
     //cout<<pre_robotPosition<<endl<<endl;

     //cout<<observedLandmark.rho<<"   "<<observedLandmark.theta<<endl;

     //std::cout<<pre_oLandmarkR<<std::endl<<endl;
    // std::cout<<lineRs.size()<<std::endl;

     Eigen::Vector2d real_oLandmarkR(0,0);
     
     bool isOn = false;  //是否能找到对应的observedLandmark
     //根据传感器数值获得实际的机器人坐标系下的observedLandmark的极坐标, 与observedlandmark相关联
     if(lineRs.size()!=0)
     {
          for(int i=0; i<lineRs.size(); i++)
          {  
                 double rhoerror = fabs( lineRs[i].rho - pre_oLandmarkR(0) );
                 double thetaerror = fabs( pi_limit(lineRs[i].theta - pre_oLandmarkR(1)) ) ; //注意要用fabs,而不是abs,因为abs会取整数
                // cout<<lineRs[i].rho<<"   "<<lineRs[i].theta<<endl;
                 //cout<<"error: "<<rhoerror<<"   "<<thetaerror<<endl<<endl<<endl;

                 if( (rhoerror < rhoThreshold*pre_oLandmarkR(0)) && (thetaerror < thetaThreshold) )
                 {    
                       isOn = true;
                       real_oLandmarkR(0) = lineRs[i].rho;
                       real_oLandmarkR(1) = lineRs[i].theta;
                    
                       break;
                 }
                 else
                  {
                     isOn = false; 
		  }
          }
         

          if(isOn)
          {
                Eigen::Vector2d MeasurementError;
                MeasurementError= real_oLandmarkR - pre_oLandmarkR;  //测量余差
                return MeasurementError;
          }
          else
          {
                std::cout<<"找不到对应的landmark"<<std::endl; 
          }

    }
    else
    {
          std::cout<<"看不到landmark"<<std::endl; 
    }
    
}




//Landmark是世界坐标系下的表示,坐标变换为机器人坐标系下的极坐标
Eigen::Vector2d ekfslam::WtoR(Landmark landmark, Eigen::Vector3d robotPosition_)
{
        double rhok = sqrt( pow(robotPosition_(0), 2)+pow(robotPosition_(1), 2) ); //robotPosition(0)是机器人坐标x的值
        double thetak;
        if(robotPosition_(0)!=0)
        {
               thetak = atan2(robotPosition_(1), robotPosition_(0));
        }
        else 
        {
               if( robotPosition_(1)>0 )
                    thetak = PI/2;
               else if( robotPosition_(1)<0 )
                    thetak = -PI/2;
               else
                    thetak = 0;                  
        }
        double d = rhok*cos(landmark.theta-thetak);

        Eigen::Vector2d oLandmarkR;  //机器人坐标系下的极坐标
        oLandmarkR(0) = fabs(landmark.rho - d);

        double origin, robot;
        origin = -landmark.rho; //原点代入直线方程xcostheta+ysintheta-rho=0
        robot = robotPosition_(0)*cos(landmark.theta) + robotPosition_(1)*sin(landmark.theta) - landmark.rho;

        if(origin<0 == robot<0)      //如果同号，则原点和机器人在直线同一侧
              oLandmarkR(1) = landmark.theta - robotPosition_(2);
        else
              oLandmarkR(1) = landmark.theta - robotPosition_(2) - PI;
        
        oLandmarkR(1) = pi_limit(oLandmarkR(1));
        
        return  oLandmarkR;
                
} 




void ekfslam::update_sigmaAndstate(int old_numLandmarks)
{
       if(numLandmarks>old_numLandmarks)              //若有新的landmark更新状态stateX和协方差矩阵sigma
       {
              int detaNum = numLandmarks - old_numLandmarks;

              Eigen::MatrixXd newsigma(3+2*numLandmarks, 3+2*numLandmarks);

              Eigen::MatrixXd newsigma_mn(3+2*old_numLandmarks, 2*detaNum);
              newsigma_mn = Eigen::MatrixXd::Zero(3+2*old_numLandmarks, 2*detaNum);

              Eigen::MatrixXd newsigma_nm(2*detaNum, 3+2*old_numLandmarks);
              newsigma_nm = Eigen::MatrixXd::Zero(2*detaNum, 3+2*old_numLandmarks);

              Eigen::MatrixXd newsigma_add(2*detaNum, 2*detaNum);
              newsigma_add = Eigen::MatrixXd::Identity(2*detaNum, 2*detaNum);

              newsigma << sigma, newsigma_mn,
                          newsigma_nm, newsigma_add;

              sigma = newsigma;
              
              Eigen::VectorXd newstateX(3+2*numLandmarks);
              Eigen::VectorXd newstateXadd(2*detaNum);

              for(int i=old_numLandmarks, j=0; i<numLandmarks, j<detaNum; i++, j++)
              {
                    newstateXadd(2*j) = landmarks[i].rho;
                    newstateXadd(2*j+1) = landmarks[i].theta;                    
              }

              newstateX << stateX,
                           newstateXadd;
              
              stateX = newstateX;            
       }

}











void ekfslam::extractLineW(std::vector<double> ranges, std::vector<Line>& lines)
{               
		std::vector<int> samples(NUM);
                 
		std::iota (std::begin(samples), std::end(samples), 0); //用顺序递增的值赋值指定范围内的元素
		std::vector<int> samplesSave = samples;

                std::vector<Eigen::Vector2d> samplePoints;		
                std::vector<Eigen::Vector2d> consensusPoints;

		int numTrials = 0;
		
		std::vector<int>::iterator it;		                
                
		//RANSAC algorithm
		while( numTrials < MAXTRIALS && samples.size() > MINLINEPOINTS)
		{
			
                        samplePoints.clear();
                        consensusPoints.clear();
                        std::random_shuffle ( samples.begin(), samples.end() );//打乱顺序
                        int beginAngle = samples.back(); //随机角度开始的SAMPLEANGLE度内做为采样范围
                       
                        samples.pop_back();//删除已采样的点(这里是samples的最后一个元素)，防止重复采样

                        int sample_angle;
                        std::vector<Eigen::Vector2d> samplesLeft;

                        for(int i=0; i<SAMPLEANGLE; i++)
                        {
                             sample_angle=beginAngle+i;
                            
		             if(sample_angle > 359)
		             {
			            sample_angle -= 360;
		             } 
                             Eigen::Vector2d point;
                             point(0)=ranges[sample_angle];
                             point(1)=sample_angle;

                             samplesLeft.push_back(point);
                        }
                        

                        std::random_shuffle ( samplesLeft.begin(), samplesLeft.end() ); //随机打乱顺序，表示任意采样
			
			double beginx, endx;
			for(int count=0; count < MAXSAMPLE; count++)  //采样MAXSAMPLE个点
			{
				
                              if( (samplesLeft[count](0)>minRange) && (samplesLeft[count](0)<maxRange) ) //应该先排除ranges[i]=0的扫描点，及没有障碍物反射的扫描点不参与直线拟合
                              {

				      double samplex = robotPosition(0) + samplesLeft[count](0) * ( cos(samplesLeft[count](1) * PI/180 + robotPosition(2)) );
				      double sampley = robotPosition(1) + samplesLeft[count](0) * ( sin(samplesLeft[count](1) * PI/180 + robotPosition(2)) ); 
                                      Eigen::Vector2d samplepoint;
                                      samplepoint(0) = samplex;
                                      samplepoint(1) = sampley;

                                      beginx = endx = samplex;
                                      
                                      samplePoints.push_back(samplepoint);
			      }
				                                           
			}
                       
                        
			//fit line model
			double a, b, rho, theta;
			linearLeastSquares(samplePoints, a, b);
                        linearTOpolar(a, b, rho, theta);     //转换为极坐标形式

                        //std::cout<<a<<"  "<<b<<std::endl;
                        //std::cout<<rho<<"  "<<theta<<std::endl;

			int angle = 0;
			double x = 0;
			double y = 0;
                        
			// Now check distance to line of remaining readings and get consensus
			for(int i = 0; i < samples.size(); i++)
			 {
				angle = samples[i];
                                if( (ranges[angle]>minRange) && (ranges[angle])<maxRange ) //应该先排除ranges[i]=0的扫描点，及没有障碍物反射的扫描点不参与直线拟合
                                {
				      x = robotPosition(0) + ranges[angle] * ( cos(angle * PI/180 + robotPosition(2)) );
				      y = robotPosition(1) + ranges[angle] * ( sin(angle * PI/180 + robotPosition(2)) ); 
                                   
                                      beginx = min(beginx, x);
                                      endx = max(endx, x);
                                       
				      double distance = distanceToLine(a, b, x, y); 
                                      //std::cout<<distance<<std::endl;
       
				      if(distance < TOLERANCE)
				      {                                          
					    Eigen::Vector2d cpoint;
                                            cpoint(0) = x;
                                            cpoint(1) = y;
                                            consensusPoints.push_back(cpoint);
				       }
                                }
			   }

                       // std::cout<<consensusPoints.size()<<std::endl;

			if(consensusPoints.size() > CONSENSUS)
			{
				// update model
				double a=0.0, b=0.0, rho=0.0, theta=0.0;
				
				linearLeastSquares(consensusPoints, a, b);  //按引用传递了a,b的值，所以会改变
				
				Line newLine;
				newLine.a = a;
				newLine.b = b;
                                newLine.beginx = beginx;
                                newLine.endx = endx;
                                //std::cout<<a<<"  "<<b<<std::endl;

                                linearTOpolar(newLine.a, newLine.b, rho, theta);  //转换为极坐标形式
                                newLine.rho = rho;
                                newLine.theta = theta;
                                
                                newLine.consensusNum = consensusPoints.size();

                                //std::cout<<rho<<"  "<<theta<<std::endl;
                                //应该加入直线的相似性判断
                                bool different = lineCompare(newLine, lines);

                                //std::cout<<different<<std::endl;
                                                                
                                if(different && newLine.rho>0.1 && newLine.rho<3.5)
                                {
				          lines.push_back(newLine);  //加入lines
                                }
			}
                      //  else
                      //  {
			//	samples = samplesSave;  //局内点太少，不满足条件，重置计算				
			//}
                     numTrials++;
		}
               
		
}


//机器人坐标系下提取的线的方程
void ekfslam::extractLineR(std::vector<double> ranges, std::vector<Line>& lineRs)
{
                

		std::vector<int> samples(NUM);
                 
		std::iota (std::begin(samples), std::end(samples), 0); //用顺序递增的值赋值指定范围内的元素
		std::vector<int> samplesSave = samples;

                std::vector<Eigen::Vector2d> samplePoints;
		
                std::vector<Eigen::Vector2d> consensusPoints;

		int numTrials = 0;
		
		std::vector<int>::iterator it;
		
                
                
		//RANSAC algorithm
		while( numTrials < MAXTRIALS && samples.size() > MINLINEPOINTS)
		{
			
                        samplePoints.clear();
                        consensusPoints.clear();
                        std::random_shuffle ( samples.begin(), samples.end() );//打乱顺序
                        int beginAngle = samples.back(); //随机角度开始的SAMPLEANGLE度内做为采样范围
                       
                        samples.pop_back();//删除已采样的点(这里是samples的最后一个元素)，防止重复采样

                        int sample_angle;
                        std::vector<Eigen::Vector2d> samplesLeft;

                        for(int i=0; i<SAMPLEANGLE; i++)
                        {
                             sample_angle=beginAngle+i;
                            
		             if(sample_angle > 359)
		             {
			            sample_angle -= 360;
		             } 
                             Eigen::Vector2d point;
                             point(0)=ranges[sample_angle];
                             point(1)=sample_angle;

                             samplesLeft.push_back(point);
                        }
                        

                        std::random_shuffle ( samplesLeft.begin(), samplesLeft.end() ); //随机打乱顺序，表示任意采样
			
			
			for(int count=0; count < MAXSAMPLE; count++)  //采样MAXSAMPLE个点
			{				
                                if( (samplesLeft[count](0)>minRange) && (samplesLeft[count](0)<maxRange) ) //应该先排除ranges[i]=0的扫描点，及没有障碍物反射的扫描点不参与直线拟合
                                {
				      double samplex = samplesLeft[count](0) * ( cos(samplesLeft[count](1) * PI / 180) );
				      double sampley = samplesLeft[count](0) * ( sin(samplesLeft[count](1) * PI / 180) ); 
                                      Eigen::Vector2d samplepoint;
                                      samplepoint(0) = samplex;
                                      samplepoint(1) = sampley;
                                      
                                      samplePoints.push_back(samplepoint);
				}
				                                              
			 }
                       
                        
			 //fit line model
			 double a, b, rho, theta;
			 linearLeastSquares(samplePoints, a, b); 
                         linearTOpolar(a, b, rho, theta);  //转换为极坐标形式
                        
                        
			 int angle = 0;
			 double x = 0;
			 double y = 0;
                        
			 // Now check distance to line of remaining readings and get consensus
			 for(int i = 0; i < samples.size(); i++)
			 {
				angle = samples[i];
                                if( (ranges[angle]>minRange) && (ranges[angle])<maxRange ) //应该先排除ranges[i]=0的扫描点，及没有障碍物反射的扫描点不参与直线拟合
                                {
				           x = ranges[angle] * ( cos(angle * PI / 180) );
				           y = ranges[angle] * ( sin(angle * PI / 180) );   //因为opencv是左手坐标系，而机器人是右手坐标系，要做变换，取反
                                       
				           double distance = distanceToLine(a, b, x, y); 
                                 
       
				           if(distance < TOLERANCE)
				           {                                          
					       Eigen::Vector2d cpoint;
                                               cpoint(0) = x;
                                               cpoint(1) = y;
                                               consensusPoints.push_back(cpoint);
				           }
                                }
			  }


			  if(consensusPoints.size() > CONSENSUS)
			  {
				// update model
				double a=0.0, b=0.0, rho=0.0, theta=0.0;
				
				linearLeastSquares(consensusPoints, a, b);  //按引用传递了a,b的值，所以会改变
				
				Line newLine;
				newLine.a = a;
			        newLine.b = b;

                                linearTOpolar(newLine.a, newLine.b, rho, theta);  //转换为极坐标形式
                                newLine.rho = rho;
                                newLine.theta = theta;
                                newLine.consensusNum = consensusPoints.size();
                                //应该加入直线的相似性判断
                                bool different = lineCompare(newLine, lineRs);

                                                                
                                if(different)
                                {
				          lineRs.push_back(newLine);  //加入lines
                                }
			 }
                        // else
                        // {
			//	 samples = samplesSave;  //局内点太少，不满足条件，重置计算				
			// }

                        numTrials++;
		}
               		
}





double ekfslam::distanceToLine(double a, double b, double x, double y)
{
	float aPer, bPer;
        double distance;
		
        if(a == 0)
        {
               if( (y-b)>0 )
               { 
                     distance = y-b;
               }
               else
               {
                     distance = b-y;
               }   
		      
         }              
         else
         {
		      aPer = -1/a;
		      bPer = y - aPer * x;
		
		      float xInt = (b - bPer)/(aPer - a);
		      float yInt = (xInt * aPer) + bPer;

		      distance = sqrt(pow(xInt - x,2) + pow(yInt-y,2)); 
         }
		
		//std::cout<<a<<"   "<<distance<<std::endl;
         return distance;
}







//最小二乘法拟合直线
void ekfslam::linearLeastSquares( std::vector<Eigen::Vector2d> Points, double &a, double &b)  //Points是xy形式的坐标
{
       int n = Points.size();
        if(n<3)                //点的数量小于3,则不用拟合直线
        {
                a=0;
                b=0;
        }                       
	else
        {	
		double sumX=0.0, sumY=0.0, sumYY=0.0, sumXX=0.0, sumYX=0.0;
               
		for( int i = 0; i < n; i++)
		{
		         double x = Points[i](0); 
		         double y = Points[i](1); 			 
                         sumY += y;
			 sumYY += pow(y,2);
			 sumX += x;
 			 sumXX += pow(x,2);
 			 sumYX += y*x;                             
		}
                
                double xMean = sumX / n;
		double yMean = sumY / n; //均值

                double Nx = n*sumXX-pow(sumX, 2);
                double Ny = n*sumYY-pow(sumY, 2);
                
                if(Nx>Ny) //直线趋近与x轴，用y=ax+b拟合
                {
                        a = (n*sumYX-sumX*sumY)/Nx;
		        b = (sumY*sumXX-sumX*sumYX)/Nx;                
                }
                else     //直线趋近与y轴，用x=cy+d拟合
                {
                        double c = (n*sumYX-sumY*sumX)/Ny; 
                        double d = (sumX*sumYY-sumY*sumYX)/Ny;
                        if(c!=0)
                        {
                               a = 1/c;
                               b = -d/c;
                        } 
                        else
                        {
                               a = 1000;
                               b = yMean - xMean * a;
                        }
                }

                //NaN，是Not a Number的缩写。,infinfinite，表示“无穷大”.
                if(std::isinf(a) || std::isnan(a))
                {
			           a = 1000;
			           b = yMean - xMean * a;		    
		}
	}

}



bool ekfslam::lineCompare(Line newLine, std::vector<Line> lines) //与已经提取出的线进行对比
{
      
      if(lines.size() == 0)
      {
           return true;
      }  
      else 
      {     
            for(int i=0; i<lines.size(); i++) 
            {   
                    double rhoerror = fabs(lines[i].rho-newLine.rho);
                    double thetaerror = fabs( pi_limit(lines[i].theta-newLine.theta) );

                    //std::cout<<rhoerror<<"  "<<thetaerror<<std::endl;

                    //两直线的rho之差<rho阈值*rho 且theta之差<theta阈值*theta,则表示两个直线是同一条直线
                    if( (rhoerror < rhoThreshold) && (thetaerror < thetaThreshold) )  
                    {
                         if(newLine.consensusNum>lines[i].consensusNum)
                         {
                                 lines[i] = newLine;
                         }
                         
                         return false;
                    }   
            }        
      }  

      return true;
}





//转化直线为极坐标形式
void ekfslam::linearTOpolar(double a, double b, double &rho, double &theta)
{
         double a1 = 1+a*a;
         rho = fabs( b/sqrt(a1) );
         theta = atan2(b/a1, -a*b/a1); //由原点到直线的垂点求得极坐标角度，atan求得的角度在[-PI/2,PI/2]的范围内,而atan2求得的角度在[-PI,PI]的范围内
}


//保证角度在区间[-PI, PI]
double ekfslam::pi_limit(double angle)                    
{
    if(angle > PI)
        angle = angle - 2*PI;

    else if (angle < -PI) 
        angle = angle + 2*PI;
    else
        angle =angle;

    return angle;
}


double ekfslam::min(double x1, double x2)
{
     if(x1<x2){
        return x1;
     }
     else{
        return x2;
     }
}


double ekfslam::max(double x1, double x2)
{
     if(x1<x2){
        return x2;
     }
     else{
        return x1;
     }
}





void ekfslam::displayPL(std::vector<double> ranges)
{

     //画扫描的点和提取的直线(odom坐标系中)
      ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
      double z = 0;

      visualization_msgs::Marker points, line_list;
         
      points.header.frame_id = "map";
      points.header.stamp = ros::Time(0);
      points.ns = "points";
      points.action = visualization_msgs::Marker::ADD;
      points.pose.orientation.w = 1.0;
      points.id = 0;
      points.type = visualization_msgs::Marker::POINTS;
      points.frame_locked = true;
      // scale成员对于这些marker类型是不同的,POINTS marker分别使用x和y作为宽和高
      points.scale.x = 0.02;
      points.scale.y = 0.02;
      // 点为红色
      points.color.r = 1.0f;
      points.color.a = 1.0;

      for(int i=0; i<NUM; i++)
      {
           geometry_msgs::Point p;
           p.x = robotPosition(0) + ranges[i] * ( cos(i * PI/180 + robotPosition(2)) );;
           p.y = robotPosition(1) + ranges[i] * ( sin(i * PI/180 + robotPosition(2)) );
           p.z = z;
           points.points.push_back(p);          
      }
                           
           
    
     for(int i=0; i<numLandmarks; i++)
     {
          line_list.header.frame_id = "map";
          line_list.header.stamp = ros::Time(0);
          line_list.ns = "lines";
          line_list.action = visualization_msgs::Marker::ADD;
          line_list.pose.orientation.w = 1.0;
          line_list.id = 1;
          line_list.type = visualization_msgs::Marker::LINE_LIST;
          line_list.frame_locked = true;
          
          line_list.scale.x = 0.01;//线的宽度
          

          //线的颜色
          line_list.color.g = 1.0;
          line_list.color.a = 1.0;

          
          geometry_msgs::Point lp;
          lp.x = landmarks[i].beginx;
          lp.y = landmarks[i].a*landmarks[i].beginx+landmarks[i].b;
/*
          if( sin(landmarks[i].theta)!=0 )
             lp.y = lp.x / tan(landmarks[i].theta) + landmarks[i].rho/sin(landmarks[i].theta);
          else
             lp.y = -1;
*/
          lp.z = z;          
          line_list.points.push_back(lp);
           //cout<<p.x<<"   "<<p.y<<endl;
          
          lp.x = landmarks[i].endx;
          lp.y = landmarks[i].a*landmarks[i].endx+landmarks[i].b;
/*
          if( sin(landmarks[i].theta)!=0 )
             lp.y = lp.x / tan(landmarks[i].theta) + landmarks[i].rho/sin(landmarks[i].theta);
          else
             lp.y = 1;
*/
          lp.z = z;
          line_list.points.push_back(lp);         
     }
     marker_pub.publish(points);
     marker_pub.publish(line_list);

}




void ekfslam::publishTF(const nav_msgs::Odometry::ConstPtr& odomMsgAddress)
{
          static tf::TransformBroadcaster br; //创建一个广播对象，它会把变化信息广播到tf中

          odom_robotPosition(0) = odomMsgAddress->pose.pose.position.x;
	  odom_robotPosition(1) = odomMsgAddress->pose.pose.position.y;
      
          tf::Quaternion quat1;
          tf::quaternionMsgToTF(odomMsgAddress->pose.pose.orientation, quat1); //四元素转化为欧拉角

          // the tf::Quaternion has a method to acess roll pitch and yaw
          double roll, pitch, yaw;
          tf::Matrix3x3(quat1).getRPY(roll, pitch, yaw); 
          odom_robotPosition(2) = yaw;

          //cout<<odom_robotPosition<<endl<<endl;
          //cout<<robotPosition<<endl<<endl;

          //创建变化信息，并且初始化。
          tf::Transform transform1;
          transform1.setOrigin( tf::Vector3(odom_robotPosition(0), odom_robotPosition(1), 0.0) );   
          quat1.setRPY(0, 0, odom_robotPosition(2));
          transform1.setRotation(quat1);
           //指出这个变换信息是base_scan相对odom的坐标信息，并且广播出去
          br.sendTransform( tf::StampedTransform(transform1, ros::Time::now(), "odom", "scan") );


     /*     //创建变化信息，并且初始化。
          tf::Transform transform2;
          transform2.setOrigin( tf::Vector3(robotPosition(0), robotPosition(1), 0.0) ); 
  
          tf::Quaternion quat2;
          quat2.setRPY(0, 0, robotPosition(2));
          transform2.setRotation(quat2);
           //指出这个变换信息是base_scan相对map的坐标信息，并且广播出去
          br.sendTransform( tf::StampedTransform(transform2, ros::Time::now(), "map", "scan") );*/
          

          //odom相对map的坐标
          double relatex = robotPosition(0) - odom_robotPosition(0);
          double relatey = robotPosition(1) - odom_robotPosition(1); //这是机器人坐标系下的相对坐标

          double rm = sqrt( relatex*relatex + relatey*relatey );
          double thetam = atan2(relatey, relatex)+robotPosition(2);  //odom坐标系在map坐标系下的极坐标
          
          double odom_mapx = rm*cos(thetam);
          double odom_mapy = rm*sin(thetam);
          double odom_maptheta = robotPosition(2) - odom_robotPosition(2);
          
           
          tf::Transform transform3;
          transform3.setOrigin( tf::Vector3(odom_mapx, odom_mapy, 0.0) );  
 
          tf::Quaternion quat3;
          quat3.setRPY(0, 0, odom_maptheta);
          transform3.setRotation(quat3);
           //指出这个变换信息是base_scan相对odom的坐标信息，并且广播出去
          br.sendTransform( tf::StampedTransform(transform3, ros::Time::now(), "map", "odom") );

}
