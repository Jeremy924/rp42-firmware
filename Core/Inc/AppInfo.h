/*
 * AppInfo.h
 *
 *  Created on: Jun 22, 2025
 *      Author: Jerem
 */

#ifndef INC_APPINFO_H_
#define INC_APPINFO_H_

typedef struct AppInfo {
	uint8_t file_version;
	uint8_t version_major;
	uint8_t version_minor;
	char app_name[16];
	char app_publisher[16];
	uint8_t published_year;
	uint8_t published_month;
	uint8_t published_day;
	char home_folder[64];
};

#endif /* INC_APPINFO_H_ */
