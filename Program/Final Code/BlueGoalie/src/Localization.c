#include "Localization.h"

// distance value
#define range0 12000	// parameter 8000 6000 5000 need to be adjust for the max and second max
#define range1 8000     // Wu and  Chen 12 8 6
#define range2 6000
#define boundary_up 50
#define boundary_down -50
#define threshold_distance_slow 50

//#define Kp 100 // Red
//#define Kd 10  // Red
#define increment_uturn 1

#define Kp 200
#define Kd 12.5

unsigned int blobs[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
float position[2] = {0, 0};
float sin_theta_robo, cos_theta_robo;
float sin_ori_goal, cos_ori_goal;
float sin_robo_goal, cos_robo_goal, dist_robo_goal;

void calculate_pos(int * goal) {
	m_wii_read(&blobs[0]);
//    m_usb_tx_uint(blobs[0]);
//    m_usb_tx_string("   ");
//    m_usb_tx_uint(blobs[1]);
//    m_usb_tx_string("   ");
//    m_usb_tx_uint(blobs[3]);
//    m_usb_tx_string("   ");
//    m_usb_tx_uint(blobs[4]);
//    m_usb_tx_string("   ");
//    m_usb_tx_uint(blobs[6]);
//    m_usb_tx_string("   ");
//    m_usb_tx_uint(blobs[7]);
//    m_usb_tx_string("   ");
//    m_usb_tx_uint(blobs[9]);
//    m_usb_tx_string("   ");
//    m_usb_tx_uint(blobs[10]);
//    m_usb_tx_string("\n");
	unsigned int x[4];		// coordination x
	unsigned int y[4];		// coordination y
	int length = 0;
    int i;
	for (i = 0; i < 4; i++)
	{
		if ((blobs[3 * i] != 1023) && (blobs[3 * i + 1] != 1023))	// to read reliable data
		{
			x[length] = blobs[3 * i];
			y[length] = blobs[3 * i + 1];
			length++;
		}
	}

	if ( length >= 3)									// only work with more than 3 points location readable
	{
		int dist[(length - 1) * length / 2];
		int point[(length - 1) * length / 2][2];		// # of point, not coordination
		int j, k = 0, m, c1, c2;
		int x_max1, x_max2, y_max1, y_max2;
		int c3;
		float len_max12;
		for ( i = 0; i < (length - 1) ; i++)
		{
			for (j = i + 1; j < length; j++)
			{
				point[k][0] = i;
				point[k][1] = j;
				dist[k] = ((x[i] - x[j]) * (x[i] - x[j])) + ((y[i] - y[j]) * (y[i] - y[j]));
				m = k;
//                m_usb_tx_ulong(dist[k]);
//                m_usb_tx_string("    ");
				while((m > 0) && (dist[m] > dist[m - 1]))			// bubble sort, high to low
				{
					c1 = point[m - 1][0];
					c2 = point[m - 1][1];
					c3 = dist[m - 1];
					point[m - 1][0] = point[m][0];
					point[m - 1][1] = point[m][1];
					dist[m - 1] = dist[m];
					point[m ][0] = c1;
					point[m][1] = c2;
					dist[m] = c3;
					m--;
				}
				k++;
			}
		}
        m_usb_tx_string("\n");
		if ( dist[0] > range0)
		{
			m_usb_tx_string("max distance out of range");
			m_usb_tx_ulong(dist[0]);
            m_usb_tx_string("\n");
            
		} 
		else
		{
			if (dist[0] > range1)		// set point 1 as the bottom, 2 as the top
			{
				if ((point[0][0] == point[1][0]) || (point[0][0] == point[1][1]))
				{
					x_max1 = x[point[0][0]];
					y_max1 = y[point[0][0]];
					x_max2 = x[point[0][1]];
					y_max2 = y[point[0][1]];
				}
				else
				{
					x_max1 = x[point[0][1]];
					y_max1 = y[point[0][1]];
					x_max2 = x[point[0][0]];
					y_max2 = y[point[0][0]];
				}
			} 
			else if (dist[0] > range2)		// set point 3 as the left, 4 as the right
			{								// case only 1\3\4 exist
				x_max1 = x[point[0][0] + point[0][1] + point[1][0] + point[1][1] - 3];
				y_max1 = y[point[0][0] + point[0][1] + point[1][0] + point[1][1] - 3];
				int ind_4 = point[0][0] + point[0][1] + point[2][0] + point[2][1] - 3;
				int ind_3 = point[1][0] + point[1][1] + point[2][0] + point[2][1] - 3;
				x_max2 = x_max1 + 3.2073 * (0.4754 * (x[ind_4] - x_max1) + 0.5246 * (x[ind_3] - x_max1));
				y_max2 = y_max1 + 3.2073 * (0.4754 * (y[ind_4] - y_max1) + 0.5246 * (y[ind_3] - y_max1));
			} 
			else							// case only 2\3\4 exist
			{
				x_max2 = x[point[1][0] + point[1][1] + point[2][0] + point[2][1] - 3];
				y_max2 = y[point[1][0] + point[1][1] + point[2][0] + point[2][1] - 3];
				int ind_3 = point[0][0] + point[0][1] + point[1][0] + point[1][1] - 3;
				int ind_4 = point[0][0] + point[0][1] + point[2][0] + point[2][1] - 3;
				x_max1 = x_max2 + 1.4530 * (0.4754 * (x[ind_4] - x_max2) + 0.5246 * (x[ind_3] - x_max2));
				y_max1 = y_max2 + 1.4530 * (0.4754 * (y[ind_4] - y_max2) + 0.5246 * (y[ind_3] - y_max2));
			}

			len_max12 = sqrt((y_max2 - y_max1) * (y_max2 - y_max1) + (x_max2 - x_max1) * (x_max2 - x_max1));
			cos_theta_robo = (y_max2 - y_max1) / len_max12;			// theta_robo = pi / 2 - theta1
			sin_theta_robo = (x_max2 - x_max1) / len_max12;
			position[0] = (1024 - x_max1 - x_max2) / 2 * cos_theta_robo - (768 - y_max1 - y_max2) / 2 * sin_theta_robo;
			position[1] = (1024 - x_max1 - x_max2) / 2 * sin_theta_robo + (768 - y_max1 - y_max2) / 2 * cos_theta_robo;
			
			dist_robo_goal = sqrt((position[0] - goal[0]) * (position[0] - goal[0]) + (position[1] - goal[1]) * (position[1] - goal[1]));
			cos_robo_goal = (goal[0] - position[0]) / dist_robo_goal;
			sin_robo_goal = (goal[1] - position[1]) / dist_robo_goal;
			sin_ori_goal = sin_theta_robo * cos_robo_goal - cos_theta_robo * sin_robo_goal;
			cos_ori_goal = cos_theta_robo * cos_robo_goal + sin_theta_robo * sin_robo_goal;
			
            m_usb_tx_string("goal[0]:  ");
            m_usb_tx_int(goal[0]);
            m_usb_tx_string("   goal[1]:  ");
            m_usb_tx_int(goal[1]);
            m_usb_tx_string("\n");
            m_usb_tx_string("x_position:  ");
			m_usb_tx_int(position[0]);
            m_usb_tx_string("   y_position:  ");
			m_usb_tx_int(position[1]);
            m_usb_tx_string("\n");
            m_usb_tx_string("   sin_theta_robo:  ");
			m_usb_tx_int(sin_theta_robo * 10000);
            m_usb_tx_string("   cos_theta_robo:  ");
			m_usb_tx_int(cos_theta_robo * 10000);
            m_usb_tx_string("\n");
            m_usb_tx_string("   sin_robo_goal:  ");
            m_usb_tx_int(sin_robo_goal * 10000);
            m_usb_tx_string("   cos_robo_goal:  ");
            m_usb_tx_int(cos_robo_goal * 10000);
            m_usb_tx_string("\n");
            m_usb_tx_string("   sin_ori_goal:  ");
            m_usb_tx_int(sin_ori_goal * 10000);
            m_usb_tx_string("   cos_ori_goal:  ");
            m_usb_tx_int(cos_ori_goal * 10000);
            m_usb_tx_string("\n");
		}

	}
	else
	{
//		m_usb_tx_string("less than 3 points: ");
//		m_usb_tx_int(length);
//        m_usb_tx_string("\n");
	}
}

void go_for_goal(int * goal) {
    calculate_pos(goal);
	static float sin_ori_goal_old = 0.0;
    if (dist_robo_goal < 10) {
        OCR1C = 0;
        OCR1B = 0;
    } else {
        if (sin_ori_goal < 0)
        {
            if ((cos_ori_goal < 0) || (sin_ori_goal < -0.712))	// Right U-turn
            {
                OCR1C = 0.8 * OCR1A;    // Red 1    Blue 0.8   difference change
                OCR1B = 0.8 * OCR1A;    // Red 0.5  Blue 0.4
                set(PORTB, 2);
                clear(PORTB, 3);
//                OCR1C += increment_uturn;
//                OCR1B -= increment_uturn;
//                if (OCR1C > OCR1A) {
//                    OCR1C = OCR1A;
//                }
//                if (OCR1B < 1) {
//                    OCR1B = 1;
//                }
            }
            else
            {
                OCR1C =  OCR1A;
                OCR1B =  OCR1A + Kp * sin_ori_goal + Kd * (sin_ori_goal - sin_ori_goal_old);
                set(PORTB, 2);		// only move forward
                set(PORTB, 3);
            }
            m_usb_tx_string("Right turn   ");
        }
        else
        {
            if ((cos_ori_goal < 0) || (sin_ori_goal > 0.712))	// Left U-turn
            {
                OCR1B = 0.8 * OCR1A;    // Red 1    Blue 0.8
                OCR1C = 0.8 * OCR1A;    // Red 0.5  Blue 0.4
                clear(PORTB, 3);
                set(PORTB, 2);
//                OCR1C -= increment_uturn;
//                OCR1B += increment_uturn;
//                if (OCR1C < 1) {
//                    OCR1C = 1;
//                }
//                if (OCR1B > OCR1A) {
//                    OCR1B = OCR1A;
//                }
            }
            else
            {
                OCR1B = OCR1A;
                OCR1C = OCR1A - Kp * sin_ori_goal - Kd * (sin_ori_goal - sin_ori_goal_old);
                set(PORTB, 2);		// only move forward
                set(PORTB, 3);
            }
            m_usb_tx_string("Left turn   ");
        }
    }
	sin_ori_goal_old = sin_ori_goal;
    m_usb_tx_string("OCR1C:   ");
    m_usb_tx_int(OCR1C);
    m_usb_tx_string("      OCR1B:   ");
    m_usb_tx_int(OCR1B);
    m_usb_tx_string("\n");
}

char compensator_indicator() {
    if ((cos_robo_goal * cos_theta_robo < 0.2)) {
        if ((position[1] < boundary_up) && (position[1] > boundary_down)) {
            return 1;
        }
        else if (position[1] * cos_robo_goal > 0) { // turn right
            return 2;
        }
        else {                                      // turn left
            return 3;
        }
    }
    return 0;
}

char go_to_goal(int * goal) {
    calculate_pos(goal);
    static float sin_ori_goal_old = 0.0;
    if (sin_ori_goal < 0)
    {
        if ((cos_ori_goal < 0) || (sin_ori_goal < -0.712))	// Right U-turn
        {
            OCR1C = 0.8 * OCR1A;    // Red 1    Blue 0.8   difference change
            OCR1B = 0.8 * OCR1A;    // Red 0.5  Blue 0.4
            set(PORTB, 2);
            clear(PORTB, 3);
            //                OCR1C += increment_uturn;
            //                OCR1B -= increment_uturn;
            //                if (OCR1C > OCR1A) {
            //                    OCR1C = OCR1A;
            //                }
            //                if (OCR1B < 1) {
            //                    OCR1B = 1;
            //                }
        }
        else
        {
            if (dist_robo_goal > threshold_distance_slow) {
                OCR1C =  OCR1A;
                OCR1B =  OCR1A + Kp * sin_ori_goal + Kd * (sin_ori_goal - sin_ori_goal_old);
            } else {
                OCR1C =  0.8 / threshold_distance_slow * dist_robo_goal * OCR1A;
                OCR1B =  OCR1C + Kp * sin_ori_goal + Kd * (sin_ori_goal - sin_ori_goal_old);
            }
            set(PORTB, 2);		// only move forward
            set(PORTB, 3);
        }
        m_usb_tx_string("Right turn   ");
    }
    else
    {
        if ((cos_ori_goal < 0) || (sin_ori_goal > 0.712))	// Left U-turn
        {
            OCR1B = 0.8 * OCR1A;    // Red 1    Blue 0.8
            OCR1C = 0.8 * OCR1A;    // Red 0.5  Blue 0.4
            clear(PORTB, 3);
            set(PORTB, 2);
            //                OCR1C -= increment_uturn;
            //                OCR1B += increment_uturn;
            //                if (OCR1C < 1) {
            //                    OCR1C = 1;
            //                }
            //                if (OCR1B > OCR1A) {
            //                    OCR1B = OCR1A;
            //                }
        }
        else
        {
            if (dist_robo_goal > threshold_distance_slow) {
                OCR1B = OCR1A;
                OCR1C = OCR1A - Kp * sin_ori_goal - Kd * (sin_ori_goal - sin_ori_goal_old);
            } else {
                OCR1B =  1.0 / threshold_distance_slow * dist_robo_goal * OCR1A;
                OCR1C =  OCR1B + Kp * sin_ori_goal + Kd * (sin_ori_goal - sin_ori_goal_old);
            }
            
            set(PORTB, 2);		// only move forward
            set(PORTB, 3);
        }
        m_usb_tx_string("Left turn   ");
    }
    sin_ori_goal_old = sin_ori_goal;
    m_usb_tx_string("OCR1C:   ");
    m_usb_tx_int(OCR1C);
    m_usb_tx_string("      OCR1B:   ");
    m_usb_tx_int(OCR1B);
    m_usb_tx_string("\n");
    if (dist_robo_goal > threshold_distance_slow) {
        return 0;
    } else {
        return 1;
    }
}
