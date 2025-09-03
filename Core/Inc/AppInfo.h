/*
 * AppInfo.h
 *
 *  Created on: Jun 22, 2025
 *      Author: Jerem
 */

#ifndef INC_APPINFO_H_
#define INC_APPINFO_H_

struct AppInfo {
	uint8_t version_major;
	uint8_t version_minor;
	char app_name[16];
	char app_publisher[16];
	uint8_t published_year;
	uint8_t published_month;
	uint8_t published_day;
};

struct AppListInfo {
	uint8_t file_version;
	char app_name[16];
	char config_folder[256];
};

#endif /* INC_APPINFO_H_ */
