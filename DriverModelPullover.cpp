 /*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Dummy version that does nothing (uses Vissim's internal model).         */
/*                                                                          */
/*  Version of 2021-02-24                                   Lukas Kautzsch  */
/*==========================================================================*/

#include "DriverModel.h"
#include <iostream>
#include <fstream>
#include <stdlib.h> 
#include <time.h>
#include <random>
#include <crtdbg.h>
#include <string>
#include <chrono>

//using std::ofstream;

//rand(time(0));
//ofstream myfile;

/*==========================================================================*/

double  desired_acceleration;
double  desired_lane_angle;
int     active_lane_change;
int     current_lane_change;
int     rel_target_lane;
double  desired_velocity;
int     turning_indicator;
int     vehicle_color        = RGB(0,0,0);


double behind_vehicle_id;
double behind2_vehicle_id;
double behind3_vehicle_id;
int behind_vehicle_category;
int behind2_vehicle_category;
int behind3_vehicle_category;
int leftbehind_vehicle_category;
int leftbehind2_vehicle_category;
int leftbehind3_vehicle_category;
int rightbehind_vehicle_category;
int rightbehind2_vehicle_category;
int rightbehind3_vehicle_category;
int behind_vehicle_type;
int behind2_vehicle_type;
int behind3_vehicle_type;
int leftbehind_vehicle_type;
int leftbehind2_vehicle_type;
int leftbehind3_vehicle_type;
int rightbehind_vehicle_type;
int rightbehind2_vehicle_type;
int rightbehind3_vehicle_type;
double vehicle_id;
double behind_vehicle_distance;
double behind2_vehicle_distance;
double behind3_vehicle_distance;
double leftbehind_vehicle_distance;
double leftbehind2_vehicle_distance;
double leftbehind3_vehicle_distance;
double rightbehind_vehicle_distance;
double rightbehind2_vehicle_distance;
double rightbehind3_vehicle_distance;
int current_vehicle_category;
int current_vehicle_type;
long current_vehicle;
double  current_velocity;
double  current_acceleration;
double  vehicle_length;
double  vehicle_x_coord;
double  vehicle_y_coord;
long  vehicle_type;
int vehicle_current_link;
int vehicle_current_lane;
double  gross_distance;
double  speed_difference;
double  ahead_vehicle_acceleration;
double  signal_distance;
double  s_star;
double  distance = 0;
double stopline_x_coord;
double stopline_y_coord;
double car_following_acceleration;
double ahead_vehicle;
double phase;
int udatest;
float udatest1;
//char udatest2[100];
double udatest2;
int udacheck;
double udatime;
int nooflanes;
double timestep;
double current_time;
double timeofcondition;
double randtime;

//unsigned seed = static_cast<unsigned> (std::chrono::system_clock::now().time_since_epoch().count());

int seed = 1;
std::default_random_engine generator(seed);
double meanpltime = 2.5; //Variable - Average time for initiating pullover
double lambda = 1/meanpltime; //Average rate of occurence
std::exponential_distribution<double> distribution(lambda);
 
/*==========================================================================*/

BOOL APIENTRY DllMain (HANDLE  hModule,
                       DWORD   ul_reason_for_call,
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
         break;
  }
  return TRUE;
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelSetValue (int    type,
                                           int    index1,
                                           int    index2,
                                           int    int_value,
                                           double double_value,
                                           char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, to <int_value>, <double_value> or             */
  /* <*string_value> (object and value selection depending on <type>).    */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_TIMESTEP               :
    case DRIVER_DATA_TIME                   :
        current_time = double_value;
      return 1;
    case DRIVER_DATA_USE_UDA                :
        if (index1 == 1)
        {

            return 1;
        }
        else if (index1 == 2)
        {
            return 1;
        }
        else if (index1 == 3)
        {
            return 1;
        }
/* doesn't use any UDAs */
                /* must return 1 for desired values of index1 if UDA values are to be sent from/to Vissim */
    case DRIVER_DATA_VEH_ID                 :
		vehicle_id = double_value;
		return 1;
    case DRIVER_DATA_VEH_LANE               :
        vehicle_current_lane = int_value;
        return 1;
    case DRIVER_DATA_VEH_ODOMETER           :
    case DRIVER_DATA_VEH_LANE_ANGLE         :
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
    case DRIVER_DATA_VEH_VELOCITY           :
    case DRIVER_DATA_VEH_ACCELERATION       :
    case DRIVER_DATA_VEH_LENGTH             :
    case DRIVER_DATA_VEH_WIDTH              :
    case DRIVER_DATA_VEH_WEIGHT             :
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
      turning_indicator = int_value;
      return 1;
    case DRIVER_DATA_VEH_CATEGORY           :	
	  current_vehicle_category = int_value;
	  return 1;
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
      desired_velocity = double_value;
      return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
    case DRIVER_DATA_VEH_Y_COORDINATE       :
    case DRIVER_DATA_VEH_Z_COORDINATE       :
    case DRIVER_DATA_VEH_REAR_X_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Y_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Z_COORDINATE  :
    case DRIVER_DATA_VEH_TYPE               :
		current_vehicle_type = int_value;
		return 1;
    case DRIVER_DATA_VEH_COLOR              :
      vehicle_color = int_value;
      return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
      return 0; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
                /* Must return 1 if these messages are to be sent from VISSIM!         */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
		current_lane_change = int_value;
		return 1;
    case DRIVER_DATA_VEH_REL_TARGET_LANE    :
    case DRIVER_DATA_VEH_INTAC_STATE        :
    case DRIVER_DATA_VEH_INTAC_TARGET_TYPE  :
    case DRIVER_DATA_VEH_INTAC_TARGET_ID    :
    case DRIVER_DATA_VEH_INTAC_HEADWAY      :
    case DRIVER_DATA_VEH_UDA                :
		//udatest2 = (char*)malloc(sizeof(string_value));
		//memcpy(udatest2, string_value,10);
		
        if (index1 == 1)
        {
            udatest = int_value;
        }
        else if (index1 == 2)
        {
            udacheck = int_value;
        }
        else if (index1 == 3)
        {
            udatime = double_value;
        }
		//udatest1 = double_value;
		//udatest2 = double_value;
		return 1;

    case DRIVER_DATA_NVEH_ID                :
		if (index1 == 0 && index2 == -1)
		{
			behind_vehicle_id = double_value;
		}
		else if (index1 == 0 && index2 == -2)
		{
			behind2_vehicle_id = double_value;
		}
		else if (index1 == 0 && index2 == -3)
		{
			behind3_vehicle_id = double_value;
		}
		return 1;
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
    case DRIVER_DATA_NVEH_DISTANCE          :
		if (index1 == 0 && index2 == -1)
		{
			behind_vehicle_distance = double_value;
		}
		else if (index1 == 0 && index2 == -2)
		{
			behind2_vehicle_distance = double_value;
		}
		else if (index1 == 0 && index2 == -3)
		{
			behind3_vehicle_distance = double_value;
		}
		else if(index1 == 1 && index2 == -1)
		{
			leftbehind_vehicle_distance = double_value;
		}
		else if (index1 == 1 && index2 == -2)
		{
			leftbehind2_vehicle_distance = double_value;
		}
		else if (index1 == 1 && index2 == -3)
		{
			leftbehind3_vehicle_distance = double_value;
		}
		else if (index1 == -1 && index2 == -1)
		{
			rightbehind_vehicle_distance = double_value;
		}
		else if (index1 == -1 && index2 == -2)
		{
			rightbehind2_vehicle_distance = double_value;
		}
		else if (index1 == -1 && index2 == -3)
		{
			rightbehind3_vehicle_distance = double_value;
		}
        return 1;

    case DRIVER_DATA_NVEH_REL_VELOCITY      :
    case DRIVER_DATA_NVEH_ACCELERATION      :
    case DRIVER_DATA_NVEH_LENGTH            :
    case DRIVER_DATA_NVEH_WIDTH             :
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
        if (index1 == 0 && index2 == -1)
        {
            behind_vehicle_category = int_value;
        }
		else if (index1 == 0 && index2 == -2)
		{
			behind2_vehicle_category = int_value;
		}
		else if (index1 == 0 && index2 == -3)
		{
			behind3_vehicle_category = int_value;
		}
		else if (index1 == 1 && index2 == -1)
		{
			leftbehind_vehicle_category = int_value;
		}
		else if (index1 == 1 && index2 == -2)
		{
			leftbehind2_vehicle_category = int_value;
		}
		else if (index1 == 1 && index2 == -3)
		{
			leftbehind3_vehicle_category = int_value;
		}
		return 1;
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
    case DRIVER_DATA_NVEH_TYPE              :
		if (index1 == 0 && index2 == -1)
		{
			behind_vehicle_type = int_value;
		}
		else if (index1 == 0 && index2 == -2)
		{
			behind2_vehicle_type = int_value;
		}
		else if (index1 == 0 && index2 == -3)
		{
			behind3_vehicle_type = int_value;
		}
		else if (index1 == 1 && index2 == -1)
		{
			leftbehind_vehicle_type = int_value;
		}
		else if (index1 == 1 && index2 == -2)
		{
			leftbehind2_vehicle_type = int_value;
		}
		else if (index1 == 1 && index2 == -3)
		{
			leftbehind3_vehicle_type = int_value;
		}
        return 1;
    case DRIVER_DATA_NVEH_UDA               :
    case DRIVER_DATA_NVEH_X_COORDINATE      :
    case DRIVER_DATA_NVEH_Y_COORDINATE      :
    case DRIVER_DATA_NVEH_Z_COORDINATE      :
    case DRIVER_DATA_NVEH_REAR_X_COORDINATE :
    case DRIVER_DATA_NVEH_REAR_Y_COORDINATE :
    case DRIVER_DATA_NVEH_REAR_Z_COORDINATE :
    case DRIVER_DATA_NO_OF_LANES            :
		nooflanes = int_value;
		return 1;
    case DRIVER_DATA_LANE_WIDTH             :
    case DRIVER_DATA_LANE_END_DISTANCE      :
    case DRIVER_DATA_CURRENT_LANE_POLY_N    :
    case DRIVER_DATA_CURRENT_LANE_POLY_X    :
    case DRIVER_DATA_CURRENT_LANE_POLY_Y    :
    case DRIVER_DATA_CURRENT_LANE_POLY_Z    :
    case DRIVER_DATA_RADIUS                 :
    case DRIVER_DATA_MIN_RADIUS             :
    case DRIVER_DATA_DIST_TO_MIN_RADIUS     :
    case DRIVER_DATA_SLOPE                  :
    case DRIVER_DATA_SLOPE_AHEAD            :
    case DRIVER_DATA_SIGNAL_DISTANCE        :
    case DRIVER_DATA_SIGNAL_STATE           :
    case DRIVER_DATA_SIGNAL_STATE_START     :
    case DRIVER_DATA_SPEED_LIMIT_DISTANCE   :
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
    case DRIVER_DATA_PRIO_RULE_DISTANCE     :
    case DRIVER_DATA_PRIO_RULE_STATE        :
    case DRIVER_DATA_ROUTE_SIGNAL_DISTANCE  :
    case DRIVER_DATA_ROUTE_SIGNAL_STATE     :
    case DRIVER_DATA_ROUTE_SIGNAL_CYCLE     :
      return 1;
    case DRIVER_DATA_CONFL_AREAS_COUNT      :
      return 0;  /* (to avoid getting sent lots of conflict area data) */
    case DRIVER_DATA_CONFL_AREA_TYPE        :
    case DRIVER_DATA_CONFL_AREA_YIELD       :
    case DRIVER_DATA_CONFL_AREA_DISTANCE    :
    case DRIVER_DATA_CONFL_AREA_LENGTH      :
    case DRIVER_DATA_CONFL_AREA_VEHICLES    :
    case DRIVER_DATA_CONFL_AREA_TIME_ENTER  :
    case DRIVER_DATA_CONFL_AREA_TIME_IN     :
    case DRIVER_DATA_CONFL_AREA_TIME_EXIT   :
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION   :
      desired_acceleration = double_value;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE     :
      desired_lane_angle = double_value;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE     :
      active_lane_change = int_value;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE        :
      rel_target_lane = int_value;
      return 1;
    default :
      return 0;
  }
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelSetValue3 (int    type,
                                            int    index1,
                                            int    index2,
                                            int    index3,
                                            int    int_value,
                                            double double_value,
                                            char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1>, */
  /* <index2> and <index3>, to <int_value>, <double_value> or              */
  /* <*string_value> (object and value selection depending on <type>).     */
  /* Return value is 1 on success, otherwise 0.                            */
  /* DriverModelGetValue (DRIVER_DATA_MAX_NUM_INDICES, ...) needs to set   */
  /* *int_value to 3 or greater in order to activate this function!        */

  switch (type) {
    case DRIVER_DATA_ROUTE_SIGNAL_SWITCH:
      return 0; /* don't send any more switch values */
    default:
      return 0;
  }
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue (int    type,
                                           int    index1,
                                           int    index2,
                                           int    *int_value,
                                           double *double_value,
                                           char   **string_value)
{
  /* Gets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, and writes that value to <*int_value>,        */
  /* <*double_value> or <**string_value> (object and value selection      */
  /* depending on <type>).                                                */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_STATUS :
      *int_value = 0;
      return 1;
    case DRIVER_DATA_WANTS_ALL_SIGNALS:
      *int_value = 1; /* needs to be set to zero if no global signal data is required */
      return 1;
    case DRIVER_DATA_MAX_NUM_INDICES:
      *int_value = 3; /* because DriverModelSetValue3() and DriverModelSetValue3() exist in this DLL */
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
      *int_value = turning_indicator;
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
      *double_value = desired_velocity;
      return 1;
    case DRIVER_DATA_VEH_COLOR :
      *int_value = vehicle_color;
      return 1;
    case DRIVER_DATA_VEH_UDA :
        if (index1 == 1)
        {
            *int_value = udatest;
        }
        else if (index1 == 2)
        {
            *int_value = udacheck;
        }
        else if (index1 == 3)
        {
            *double_value = udatime;
        }
      return 1; /* doesn't set any UDA values */
    case DRIVER_DATA_WANTS_SUGGESTION :
      *int_value = 1;
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
      *double_value = desired_acceleration;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
      *double_value = desired_lane_angle;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      *int_value = active_lane_change;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
      *int_value = rel_target_lane;
      return 1;
    case DRIVER_DATA_SIMPLE_LANECHANGE :
      *int_value = 1;
      return 1;
    case DRIVER_DATA_USE_INTERNAL_MODEL:
		*int_value = 0;/* must be set to 0 if external model is to be applied */
      return 1;
    case DRIVER_DATA_WANTS_ALL_NVEHS:
      *int_value = 1; /* must be set to 1 if data for more than 2 nearby vehicles per lane and upstream/downstream is to be passed from Vissim */
      return 1;
    case DRIVER_DATA_ALLOW_MULTITHREADING:
      *int_value = 0; /* must be set to 1 to allow a simulation run to be started with multiple cores used in the simulation parameters */
      return 1;
    default:
      return 0;
  }
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue3 (int    type,
                                            int    index1,
                                            int    index2,
                                            int    index3,
                                            int    *int_value,
                                            double *double_value,
                                            char   **string_value)
{
  /* Gets the value of a data object of type <type>, selected by <index1>, */
  /* <index2> and <index3>, and writes that value to <*int_value>,         */
  /* <*double_value> or <**string_value> (object and value selection       */
  /* depending on <type>).                                                 */
  /* Return value is 1 on success, otherwise 0.                            */
  /* DriverModelGetValue (DRIVER_DATA_MAX_NUM_INDICES, ...) needs to set   */
  /* *int_value to 3 or greater in order to activate this function!        */

  switch (type) {
    default:
      return 0;
    }
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (int  number)
{
  /* Executes the command <number> if that is available in the driver */
  /* module. Return value is 1 on success, otherwise 0.               */

  switch (number) {
    case DRIVER_COMMAND_INIT :
      return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
      return 1;
    case DRIVER_COMMAND_KILL_DRIVER :
      return 1;
	case DRIVER_COMMAND_MOVE_DRIVER :
		
		/*
		
		myfile.open("data.txt", std::ios_base::app);
		myfile << current_vehicle_category  << "\n";
		myfile <<vehicle_current_lane << "Next";
		myfile.close();
		*/
		// behind_vehicle_category == 2 || behind2_vehicle_category == 2 || behind3_vehicle_category == 2)

		/*if (vehicle_current_lane == 1)
		{
			desired_acceleration = -9.8/10;
		}
		*/
		/*if (vehicle_current_lane == 1)
		{
		
			desired_acceleration = -9.8 / 10;
		}
		*/
		if ((leftbehind_vehicle_type == 630 && leftbehind_vehicle_distance  > -100) || (leftbehind2_vehicle_type == 630 && leftbehind2_vehicle_distance  > -100) || (leftbehind3_vehicle_type == 630 && leftbehind3_vehicle_distance  > -100))
		{

			active_lane_change = 0;
		}
		
		/*std::default_random_engine de(time(0)); //seed
		std::normal_distribution<int> nd(70, 10); //mean followed by stdiv
		int rarrary[2]; // [0, 100]
		for (int i = 0; i < 3; ++i) {
		rarray[i] = nd(de); //Generate numbers;
		}*/
        if (udacheck != 1 && udatest != 1)
        {
            if (behind_vehicle_type == 630 && behind_vehicle_distance > -45 && vehicle_current_lane != 1)
            {
                /*myfile.open("data.txt", std::ios_base::app);
                myfile << "Behind" << behind_vehicle_distance << "\n";
                myfile.close();*/
                udacheck = 1;
                udatime = current_time;
                udatest = 1;

            }
            else if (behind2_vehicle_type == 630 && behind2_vehicle_distance > -45 && vehicle_current_lane != 1)
            {
                /* myfile.open("data.txt", std::ios_base::app);
                myfile << "Behind 2" << behind2_vehicle_distance << "\n";
                myfile.close();*/
                udacheck = 1;
                udatime = current_time;
                udatest = 1;
            }
            else if (behind3_vehicle_type == 630 && behind3_vehicle_distance > -45 && vehicle_current_lane != 1)
            {
                /*myfile.open("data.txt", std::ios_base::app);
                myfile << "behind 3" << behind3_vehicle_distance << "\n";
                myfile.close();*/
                udacheck = 1;
                udatime = current_time;
                udatest = 1;
            }
        }

        //srand((unsigned)time(NULL));
        //randtime =1+ rand() % 10;


        randtime = distribution(generator);

        if (randtime > 15)
        {
            randtime = 15;
        }
        else if (randtime < 2.5)
        {
            randtime = 2.5;
        }
        

        if (current_velocity < 1.4 && udacheck == 1 && vehicle_current_lane != 1)
        {
            if ((current_time - udatime) > abs(randtime) && udacheck == 1)
            {
                udacheck = 0;
                active_lane_change = -1;
            }


        }
        if (current_velocity >= 1.4 && udacheck == 1 && vehicle_current_lane != 1)
        {
            udacheck = 0;
            active_lane_change = -1;


        }

		/*if (udatest == 1)
        * 
		{
			if (((leftbehind_vehicle_type == NULL|| leftbehind_vehicle_type != 630) && (leftbehind2_vehicle_type == NULL || leftbehind2_vehicle_type != 630) && (leftbehind3_vehicle_type == NULL || leftbehind3_vehicle_type != 630) && (behind_vehicle_type == NULL || behind_vehicle_type != 630) && (behind2_vehicle_type == NULL || behind2_vehicle_type != 630) && (behind3_vehicle_type = NULL || behind3_vehicle_type != 2)) && (vehicle_current_lane != nooflanes))
			{
				
				udatest = 99;
				active_lane_change = 1;


			}
		}
        */



		return 1;
	default:
		return 0;
  }
}

/*==========================================================================*/
/*  End of DriverModel.cpp                                                  */
/*==========================================================================*/
