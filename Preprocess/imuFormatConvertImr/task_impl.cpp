#define _CRT_SECURE_NO_WARNINGS

#pragma once
#include <iostream>
#define G_PI        3.14159265358979311599796346854419e0    // pi

int task_xwyd_decode()
{
	enum XW_MSG_TYPE
	{
		GTIMU_BIN,
		RAWIMUB
	};

	FILE* f_in = fopen("D:/Data/shiyan1027/starneto_10_27_collect.txt", "rb");
	FILE* f_out = fopen("D:/Data/shiyan1027/starneto_10_27_collect.txt.txt", "wt");
	XW_MSG_TYPE mode = XW_MSG_TYPE::RAWIMUB;

	const int BUFFER_LENGTH = 2000;
	const int MARGIN = 500;
	unsigned char chardata[BUFFER_LENGTH];

	int ptr = 0;

	if (mode == GTIMU_BIN)
	{
		while (fread(chardata + ptr, sizeof(char), BUFFER_LENGTH - ptr, f_in) == BUFFER_LENGTH - ptr)
		{
			//for (int i = 0; i < BUFFER_LENGTH; i++) printf("%02X", chardata[i]);
			//printf("\n");
			const double g = 9.80144145;
			ptr = 0;
			while (ptr < BUFFER_LENGTH - MARGIN)
			{
				if (chardata[ptr] == 0xAA && chardata[ptr + 1] == 0x55 && chardata[ptr + 2] == 0x05)
				{
					ptr += 3;

					int byten = 0;
					//printcharbit((char*)chardata, (ptr + 3) * 8, 16);
					uint16_t    week ;
					uint32_t    sow  ;
					double      gx   ;
					double      gy   ;
					double      gz   ;
					double      ax   ;
					double      ay   ;
					double      az   ;

					byten = sizeof(week    ); memcpy(&week    , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(sow     ); memcpy(&sow     , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(gx      ); memcpy(&gx      , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(gy      ); memcpy(&gy      , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(gz      ); memcpy(&gz      , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(ax      ); memcpy(&ax      , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(ay      ); memcpy(&ay      , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(az      ); memcpy(&az      , chardata + ptr, byten); ptr += byten; 

					//if (fabs(sow / 1000.0 - 443084.360000) < 1e-6)
					//{
					//	printf("???\n");
					//	getchar();
					//}

					ax *= g;
					ay *= g;
					az *= g;
					//printf("%lf %13.8lf %13.8lf %13.8lf %13.8lf %13.8lf %13.8lf\n", sow / 1000.0, gx, gy, gz, ax, ay, az);

					fprintf(f_out, "%lf %13.8lf %13.8lf %13.8lf %13.8lf %13.8lf %13.8lf\n", sow/1000.0, gx, gy, gz, ax, ay, az);

				}
				ptr++;
			}
			for (int ii = ptr; ii < BUFFER_LENGTH; ii++)
			{
				chardata[ii - ptr] = chardata[ii];
			}
			ptr = BUFFER_LENGTH - ptr;
		}
	}
	else if (mode == RAWIMUB)
	{
		while (fread(chardata + ptr, sizeof(char), BUFFER_LENGTH - ptr, f_in) == BUFFER_LENGTH - ptr)
		{
			//for (int i = 0; i < BUFFER_LENGTH; i++) printf("%02X", chardata[i]);
			//printf("\n");

			const double GYRO_FACTOR = 0.1 / (3600 * 256.0);
			const double ACCE_FACTOR = 0.05 / pow(2, 15);
			const double FREQ = 100.0;
			const double RAD2DEG = 180.0 / G_PI;

			ptr = 0;
			while (ptr < BUFFER_LENGTH - MARGIN)
			{
				if (chardata[ptr] == 0xAA && chardata[ptr + 1] == 0x44 && chardata[ptr + 2] == 0x13)
				{
					int byten = 0;
					ptr += 6;

					uint16_t    week ;
					uint32_t    sow  ;
					uint32_t    week2;
					double      sow2 ;
					int32_t     state;
                    int32_t     az   ;
                    int32_t     ay   ;
                    int32_t     ax   ;
                    int32_t     gz   ;
                    int32_t     gy   ;
                    int32_t     gx   ;

					byten = sizeof(week    ); memcpy(&week    , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(sow     ); memcpy(&sow     , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(week2   ); memcpy(&week2   , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(sow2    ); memcpy(&sow2    , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(state   ); memcpy(&state   , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(az      ); memcpy(&az      , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(ay      ); memcpy(&ay      , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(ax      ); memcpy(&ax      , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(gz      ); memcpy(&gz      , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(gy      ); memcpy(&gy      , chardata + ptr, byten); ptr += byten; 
					byten = sizeof(gx      ); memcpy(&gx      , chardata + ptr, byten); ptr += byten; 

					double gyro[3] = { gx,-gy,gz };
					double accel[3] = { ax,-ay,az };

					for (int i = 0; i < 3; i++)  gyro[i] *= GYRO_FACTOR * FREQ * RAD2DEG;
					for (int i = 0; i < 3; i++) accel[i] *= ACCE_FACTOR * FREQ;

					if (fabs(sow2 - round(sow2)) < 1e-6 && ((int)sow2) % 100 == 0)
					{
						printf("%lf\n", sow2);
					}

					//printf("%d %lf %13.8lf %13.8lf %13.8lf %13.8lf %13.8lf %13.8lf\n", week2, sow2, gx, gy, gz, ax, ay, az);
					fprintf(f_out, "%13.4lf %13.8lf %13.8lf %13.8lf %13.8lf %13.8lf %13.8lf\n", sow2, gyro[0], gyro[1], gyro[2],
						accel[0], accel[1], accel[2]);

					//getchar();
				}
				ptr++;
			}
			for (int ii = ptr; ii < BUFFER_LENGTH; ii++)
			{
				chardata[ii - ptr] = chardata[ii];
			}
			ptr = BUFFER_LENGTH - ptr;

		}
	}
	//for(int i=0;i<60;i++)
	//	printf("%X", chardata[i]);

	fclose(f_in);
	fclose(f_out);

	return 0;
}

int task_txt2imr()
{
	FILE* f_in = fopen("bigimu_out.txt", "rt");
	FILE* f_out = fopen("bigimu_out.imr", "wb");

	char      szHeader            [8] = "$IMURAW";
	int8_t    blsIntelOrMotorola      = 0;
	double    dVersionNumber          = 8.7051;
	int32_t   dDeltaTheta             = 1;
	int32_t   dDeltaVelocity          = 1;
	double    dDataRateHz             = 200;
	double    dGyroScaleFactor        = 1.00e-8;
	double    dAccelScaleFactor       = 1.00e-8;
	int32_t   iUtcOrGpsTime           = 2;
	int32_t   iRcvTimeOrCorrTime      = 2;
	double    dTimeTagBias            = 0;
	char      szImuName          [32] = "STARNETO";
	uint8_t   reserved1           [4] = {0};
	char      szProgramName      [32] = "BIN_DATA_TOOLS";
	char      tCreate            [12] = "";
	bool      bLeverArmValid          = false;
	int32_t   lXoffset                = 0;
	int32_t   lYoffset                = 0;
	int32_t   lZoffset                = 0;
	int8_t    Reserverd         [354] = {0};

	fwrite(szHeader               , sizeof(szHeader           ), 1, f_out);
	fwrite(&blsIntelOrMotorola    , sizeof(blsIntelOrMotorola ), 1, f_out);
	fwrite(&dVersionNumber        , sizeof(dVersionNumber     ), 1, f_out);
	fwrite(&dDeltaTheta           , sizeof(dDeltaTheta        ), 1, f_out);
	fwrite(&dDeltaVelocity        , sizeof(dDeltaVelocity     ), 1, f_out);
	fwrite(&dDataRateHz           , sizeof(dDataRateHz        ), 1, f_out);
	fwrite(&dGyroScaleFactor      , sizeof(dGyroScaleFactor   ), 1, f_out);
	fwrite(&dAccelScaleFactor     , sizeof(dAccelScaleFactor  ), 1, f_out);
	fwrite(&iUtcOrGpsTime         , sizeof(iUtcOrGpsTime      ), 1, f_out);
	fwrite(&iRcvTimeOrCorrTime    , sizeof(iRcvTimeOrCorrTime ), 1, f_out);
	fwrite(&dTimeTagBias          , sizeof(dTimeTagBias       ), 1, f_out);
	fwrite(szImuName              , sizeof(szImuName          ), 1, f_out);
	fwrite(reserved1              , sizeof(reserved1          ), 1, f_out);
	fwrite(szProgramName          , sizeof(szProgramName      ), 1, f_out);
	fwrite(tCreate                , sizeof(tCreate            ), 1, f_out);
	fwrite(&bLeverArmValid        , sizeof(bLeverArmValid     ), 1, f_out);
	fwrite(&lXoffset              , sizeof(lXoffset           ), 1, f_out);
	fwrite(&lYoffset              , sizeof(lYoffset           ), 1, f_out);
	fwrite(&lZoffset              , sizeof(lZoffset           ), 1, f_out);
	fwrite(Reserverd              , sizeof(Reserverd          ), 1, f_out);

	const int LINE_LENGTH = 1000;
	char line[LINE_LENGTH];

	while (fgets(line, LINE_LENGTH, f_in) != 0)
	{
		double Time;
		double gyro[3];
		double accel[3];
		if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &Time, gyro, gyro + 1, gyro + 2, accel, accel + 1, accel + 2) != 7) continue;
		int32_t   gx = (int32_t)round(gyro[0]  / (dGyroScaleFactor  * dDataRateHz));
		int32_t   gy = (int32_t)round(gyro[1]  / (dGyroScaleFactor  * dDataRateHz));
		int32_t   gz = (int32_t)round(gyro[2]  / (dGyroScaleFactor  * dDataRateHz));
		int32_t   ax = (int32_t)round(accel[0] / (dAccelScaleFactor * dDataRateHz));
		int32_t   ay = (int32_t)round(accel[1] / (dAccelScaleFactor * dDataRateHz));
		int32_t   az = (int32_t)round(accel[2] / (dAccelScaleFactor * dDataRateHz));

		fwrite(&Time        , sizeof(Time          ), 1, f_out);
		fwrite(&gx          , sizeof(gx            ), 1, f_out);
		fwrite(&gy          , sizeof(gy            ), 1, f_out);
		fwrite(&gz          , sizeof(gz            ), 1, f_out);
		fwrite(&ax          , sizeof(ax            ), 1, f_out);
		fwrite(&ay          , sizeof(ay            ), 1, f_out);
		fwrite(&az          , sizeof(az            ), 1, f_out);
	}

	fclose(f_in);
	fclose(f_out);

	return 0;
}

int task_imr2txt()
{
	FILE* f_in = fopen("F:/Data/GNSS_INS_VISION20200913/INS/starneto2.imr", "rb");
	FILE* f_out = fopen("F:/Data/GNSS_INS_VISION20200913/INS/starneto2.imr.txt", "wt");

	const int BUFFER_LENGTH = 5000;
	const int MARGIN = 800;
	unsigned char chardata[BUFFER_LENGTH];

	int ptr = 0;
	bool hdr = true;

	// HEADER
	char      szHeader            [8];
	int8_t    blsIntelOrMotorola     ;
	double    dVersionNumber         ;
	int32_t   dDeltaTheta            ;
	int32_t   dDeltaVelocity         ;
	double    dDataRateHz            ;
	double    dGyroScaleFactor       ;
	double    dAccelScaleFactor      ;
	int32_t   iUtcOrGpsTime          ;
	int32_t   iRcvTimeOrCorrTime     ;
	double    dTimeTagBias           ;
	char      szImuName          [32];
	uint8_t   reserved1           [4];
	char      szProgramName      [32];
	char      tCreate            [12];
	bool      bLeverArmValid         ;
	int32_t   lXoffset               ;
	int32_t   lYoffset               ;
	int32_t   lZoffset               ;
	int8_t    Reserverd         [354];


	while (fread(chardata + ptr, sizeof(char), BUFFER_LENGTH - ptr, f_in) == BUFFER_LENGTH - ptr)
	{
		ptr = 0;
		while (ptr < BUFFER_LENGTH - MARGIN)
		{
			if (hdr)
			{
				int byten = 0;

				byten = sizeof(szHeader          ); memcpy(szHeader             , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(blsIntelOrMotorola); memcpy(&blsIntelOrMotorola  , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(dVersionNumber    ); memcpy(&dVersionNumber      , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(dDeltaTheta       ); memcpy(&dDeltaTheta         , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(dDeltaVelocity    ); memcpy(&dDeltaVelocity      , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(dDataRateHz       ); memcpy(&dDataRateHz         , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(dGyroScaleFactor  ); memcpy(&dGyroScaleFactor    , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(dAccelScaleFactor ); memcpy(&dAccelScaleFactor   , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(iUtcOrGpsTime     ); memcpy(&iUtcOrGpsTime       , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(iRcvTimeOrCorrTime); memcpy(&iRcvTimeOrCorrTime  , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(dTimeTagBias      ); memcpy(&dTimeTagBias        , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(szImuName         ); memcpy(szImuName            , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(reserved1         ); memcpy(reserved1            , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(szProgramName     ); memcpy(szProgramName        , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(tCreate           ); memcpy(tCreate              , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(bLeverArmValid    ); memcpy(&bLeverArmValid      , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(lXoffset          ); memcpy(&lXoffset            , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(lXoffset          ); memcpy(&lXoffset            , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(lXoffset          ); memcpy(&lXoffset            , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(Reserverd         ); memcpy(Reserverd            , chardata + ptr, byten); ptr += byten; 

				hdr = false;
			}
			else
			{
				int byten = 0;

				double    Time                    ;
				int32_t   gx                      ;
				int32_t   gy                      ;
				int32_t   gz                      ;
				int32_t   ax                      ;
				int32_t   ay                      ;
				int32_t   az                      ;

				byten = sizeof(Time              ); memcpy(&Time              , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(gx                ); memcpy(&gx                , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(gy                ); memcpy(&gy                , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(gz                ); memcpy(&gz                , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(ax                ); memcpy(&ax                , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(ay                ); memcpy(&ay                , chardata + ptr, byten); ptr += byten; 
				byten = sizeof(az                ); memcpy(&az                , chardata + ptr, byten); ptr += byten; 

				double gyro[3] =  { gx, gy, gz };
				double accel[3] = { ax, ay, az };

				for (int i = 0; i < 3; i++) gyro[i]  *= dGyroScaleFactor  * dDataRateHz;
				for (int i = 0; i < 3; i++) accel[i] *= dAccelScaleFactor * dDataRateHz;

				fprintf(f_out,"%15.5lf%15.6lf%15.6lf%15.6lf%15.6lf%15.6lf%15.6lf\n", Time, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
			}
		}
		for (int ii = ptr; ii < BUFFER_LENGTH; ii++)
		{
			chardata[ii - ptr] = chardata[ii];
		}
		ptr = BUFFER_LENGTH - ptr;
	}

	fclose(f_in);
	fclose(f_out);

	return 0;

}