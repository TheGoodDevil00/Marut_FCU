void quad_mode(void *argument) {
	/* USER CODE BEGIN quad_mode */
	/* Infinite loop */

	int ThrottleIdle = 1100;
	static float gx, gy, gz;

	HAL_TIM_Base_Start(&htim10);

	uint32_t last_time = micros();

	for (;;) {

		if (mode_flag == 1) {
			if (arm_flag == 1 && disarm_flag == 0) {

				uint32_t now = micros();

				if ((uint16_t) (now - last_time) >= 3000) {

					last_time = now;

					if (display_channels[6] < 1200) // channel 7
							{

						DesiredAngleRoll = 0.10f * (display_channels[0] - 1500);
						DesiredAnglePitch = 0.10f
								* (display_channels[1] - 1500);
						InputThrottle = display_channels[2];
						DesiredRateYaw = 0.15f * (display_channels[3] - 1500);
						DesiredRateYaw = -DesiredRateYaw;

						if (osMutexAcquire(i2c_mutexHandle, 0) == osOK) { //try reversing control loop tommorow, should proboably work

							mpu_get_kalman_angles(&kalman_roll, &kalman_pitch);
							gx =
									(mpu_gyro_read(0)
											- calibration_const_global_gx);
							gy =
									(mpu_gyro_read(1)
											- calibration_const_global_gy);
							gz =
									(mpu_gyro_read(2)
											- calibration_const_global_gz);

							roll_snap = kalman_roll;
							pitch_snap = kalman_pitch;
							osMutexRelease(i2c_mutexHandle);

						}
						ErrorAngleRoll = DesiredAngleRoll - kalman_roll;
						ErrorAnglePitch = DesiredAnglePitch - kalman_pitch;

						pid_equation(ErrorAngleRoll, PAngleRoll_S, IAngleRoll_S,
								DAngleRoll_S, PrevErrorAngleRoll,
								PrevItermAngleRoll);
						DesiredRateRoll = PIDReturn[0];
						PrevErrorAngleRoll = PIDReturn[1];
						PrevItermAngleRoll = PIDReturn[2];

						pid_equation(ErrorAnglePitch, PAnglePitch_S,
								IAnglePitch_S, DAnglePitch_S,
								PrevErrorAnglePitch, PrevItermAnglePitch);
						DesiredRatePitch = PIDReturn[0];
						PrevErrorAnglePitch = PIDReturn[1];
						PrevItermAnglePitch = PIDReturn[2];

						ErrorRateRoll = DesiredRateRoll - gx;
						ErrorRatePitch = DesiredRatePitch - gy;
						ErrorRateYaw = DesiredRateYaw - gz;

						pid_equation(ErrorRateRoll, PRateRoll_S, IRateRoll_S,
								DRateRoll_S, PrevErrorRateRoll,
								PrevItermRateRoll);
						InputRoll = PIDReturn[0];
						PrevErrorRateRoll = PIDReturn[1];
						PrevItermRateRoll = PIDReturn[2];

						pid_equation(ErrorRatePitch, PRatePitch_S, IRatePitch_S,
								DRatePitch_S, PrevErrorRatePitch,
								PrevItermRatePitch);
						InputPitch = PIDReturn[0];
						PrevErrorRatePitch = PIDReturn[1];
						PrevItermRatePitch = PIDReturn[2];

						pid_equation(ErrorRateYaw, PRateYaw_S, IRateYaw_S,
								DRateYaw_S, PrevErrorRateYaw, PrevItermRateYaw);
						InputYaw = PIDReturn[0];
						PrevErrorRateYaw = PIDReturn[1];
						PrevItermRateYaw = PIDReturn[2];

						M1 = 1.024f
								* (InputThrottle - InputRoll - InputPitch
										- InputYaw);
						M2 = 1.024f
								* (InputThrottle - InputRoll + InputPitch
										+ InputYaw);
						M3 = 1.024f
								* (InputThrottle + InputRoll + InputPitch
										- InputYaw);
						M4 = 1.024f
								* (InputThrottle + InputRoll - InputPitch
										+ InputYaw);

						if (M1 < ThrottleIdle)
							M1 = ThrottleIdle;
						if (M2 < ThrottleIdle)
							M2 = ThrottleIdle;
						if (M3 < ThrottleIdle)
							M3 = ThrottleIdle;
						if (M4 < ThrottleIdle)
							M4 = ThrottleIdle;

						if (M1 > 2000)
							M1 = 1999;
						if (M2 > 2000)
							M2 = 1999;
						if (M3 > 2000)
							M3 = 1999;
						if (M4 > 2000)
							M4 = 1999;

						if (display_channels[2] < 1100) {
							M1 = 1000;
							M2 = 1000;
							M3 = 1000;
							M4 = 1000;
							reset_pid();
						}

						set_raw_ccr(M1, 5);
						set_raw_ccr(M4, 7);
						set_raw_ccr(M3, 4);
						set_raw_ccr(M2, 6);

					}
					if (display_channels[6] > 1800) {

						DesiredRateRoll = 0.15f * (display_channels[0] - 1500);
						DesiredRatePitch = 0.15f * (display_channels[1] - 1500);
						InputThrottle = display_channels[2];
						DesiredRateYaw = 0.15f * (display_channels[3] - 1500);
						DesiredRateYaw = -DesiredRateYaw;

						PRatePitch_R = mapRCtoPID(display_channels[5]);
						PRateRoll_R = mapRCtoPID(display_channels[5]);
						IRateRoll_R = mapRCtoPID(display_channels[7]);
						IRatePitch_R = mapRCtoPID(display_channels[7]);

						if (osMutexAcquire(i2c_mutexHandle, 0) == osOK) {
							mpu_get_kalman_angles(&kalman_roll, &kalman_pitch);

							roll_snap = kalman_roll;
							pitch_snap = kalman_pitch;

							ErrorRateRoll = DesiredRateRoll
									- (mpu_gyro_read(0)
											- calibration_const_global_gx); //x
							ErrorRatePitch = DesiredRatePitch
									- (mpu_gyro_read(1)
											- calibration_const_global_gy); //y
							ErrorRateYaw = DesiredRateYaw
									- (mpu_gyro_read(2)
											- calibration_const_global_gz); //z
							osMutexRelease(i2c_mutexHandle);
						}

						pid_equation(ErrorRateRoll, PRateRoll_R, IRateRoll_R,
								DRateRoll_R, PrevErrorRateRoll,
								PrevItermRateRoll);
						InputRoll = PIDReturn[0];
						PrevErrorRateRoll = PIDReturn[1];
						PrevItermRateRoll = PIDReturn[2];

						pid_equation(ErrorRatePitch, PRatePitch_R, IRatePitch_R,
								DRatePitch_R, PrevErrorRatePitch,
								PrevItermRatePitch);
						InputPitch = PIDReturn[0];
						PrevErrorRatePitch = PIDReturn[1];
						PrevItermRatePitch = PIDReturn[2];

						pid_equation(ErrorRateYaw, PRateYaw_R, IRateYaw_R,
								DRateYaw_R, PrevErrorRateYaw, PrevItermRateYaw);
						InputYaw = PIDReturn[0];
						PrevErrorRateYaw = PIDReturn[1];
						PrevItermRateYaw = PIDReturn[2];

						M1 = 1.024f
								* (InputThrottle - InputRoll - InputPitch
										- InputYaw);
						M2 = 1.024f
								* (InputThrottle - InputRoll + InputPitch
										+ InputYaw);
						M3 = 1.024f
								* (InputThrottle + InputRoll + InputPitch
										- InputYaw);
						M4 = 1.024f
								* (InputThrottle + InputRoll - InputPitch
										+ InputYaw);

						if (M1 < ThrottleIdle)
							M1 = ThrottleIdle;
						if (M2 < ThrottleIdle)
							M2 = ThrottleIdle;
						if (M3 < ThrottleIdle)
							M3 = ThrottleIdle;
						if (M4 < ThrottleIdle)
							M4 = ThrottleIdle;

						if (M1 > 2000)
							M1 = 1999;
						if (M2 > 2000)
							M2 = 1999;
						if (M3 > 2000)
							M3 = 1999;
						if (M4 > 2000)
							M4 = 1999;

						if (display_channels[2] < 1100) {
							M1 = 1000;
							M2 = 1000;
							M3 = 1000;
							M4 = 1000;
							reset_pid();
						}

						set_raw_ccr(M1, 5);
						set_raw_ccr(M4, 7);
						set_raw_ccr(M3, 4);
						set_raw_ccr(M2, 6);

					}

				}

			}

		}

		/* printf("VVC is %f\n", vertical_velocity());

		 printf("P Roll and pitch is %f and I Roll and pitch is %f\n",
		 PRatePitch, IRatePitch);

		 printf("CH: "
		 "0=%4u "
		 "1=%4u "
		 "2=%4u "
		 "3=%4u "
		 "4=%4u "
		 "5=%4u "
		 "6=%4u\r\n", display_channels[0], display_channels[1],
		 display_channels[2], display_channels[3],
		 display_channels[4], display_channels[5],
		 display_channels[6]);

		 printf("M1:%lu  M2:%lu  M3:%lu  M4:%lu\r\n", (unsigned long) M1,
		 (unsigned long) M2, (unsigned long) M3, (unsigned long) M4);

		 printf("GYRO raw: gx=%+.2f gy=%+.2f gz=%+.2f\n",
		 mpu_gyro_read(0) - calibration_const_global_gx,
		 mpu_gyro_read(1) - calibration_const_global_gy,
		 mpu_gyro_read(2) - calibration_const_global_gz);

		 printf("ERR: eR=%+.2f eP=%+.2f eY=%+.2f\n", ErrorRateRoll,
		 ErrorRatePitch, ErrorRateYaw);

		 printf("PID R: P=%+.2f I=%+.2f D=%+.2f OUT=%+.2f\n",
		 PRateRoll * PrevErrorRateRoll, PrevItermRateRoll,
		 InputRoll
		 - (PRateRoll * PrevErrorRateRoll + PrevItermRateRoll),
		 InputRoll);

		 printf("PID P: P=%+.2f I=%+.2f D=%+.2f OUT=%+.2f\n",
		 PRatePitch * PrevErrorRatePitch, PrevItermRatePitch,
		 InputPitch
		 - (PRatePitch * PrevErrorRatePitch
		 + PrevItermRatePitch), InputPitch);

		 printf("PID Y: P=%+.2f I=%+.2f D=%+.2f OUT=%+.2f\n",
		 PRateYaw * PrevErrorRateYaw, PrevItermRateYaw,
		 InputYaw - (PRateYaw * PrevErrorRateYaw + PrevItermRateYaw),
		 InputYaw);

		 printf("MIX raw: M1=%d M2=%d M3=%d M4=%d\n", M1, M2, M3, M4);

		 printf("PWM OUT: TR=%d TL=%d BL=%d BR=%d\n", M1, M2, M3, M4);

		 printf("CH: "
		 "0=%4u "
		 "1=%4u "
		 "2=%4u "
		 "3=%4u "
		 "4=%4u "
		 "5=%4u "
		 "6=%4u\r\n", display_channels[0], display_channels[1],
		 display_channels[2], display_channels[3],
		 display_channels[4], display_channels[5],
		 display_channels[6]);*/


		osDelay(1);
	}

	/* USER CODE END quad_mode */
}


