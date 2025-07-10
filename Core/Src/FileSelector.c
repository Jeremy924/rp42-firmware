/*
 * FileSelector.c
 *
 *  Created on: Jun 1, 2025
 *      Author: Jerem
 */

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "FileSelector.h"
#include "fatfs.h"
#include "RP.hh"
#include "SystemIcons.h"

void draw_file_explorer_icon(uint8_t index, uint8_t page, uint8_t col) {
	const uint8_t* icon = &FILE_EXPLORER_ICONS[FILE_EXPLORER_INDICES[index]];
	uint8_t length = FILE_EXPLORER_INDICES[index + 1] - FILE_EXPLORER_INDICES[index];



	RP_DRAW_ICON(icon, length, 1, &page, &col);
}

void hide_file_explorer_icon(uint8_t index, uint8_t page, uint8_t col) {
	uint8_t length = FILE_EXPLORER_INDICES[index + 1] - FILE_EXPLORER_INDICES[index];

	uint8_t buf[length];

	memset(buf, 0b01111111, length);
	RP_DRAW_ICON(buf, length, 1, &page, &col);
}

char* get_file_name(char* full_path) {
	char* file_name_start = full_path;

	for (char* ptr = full_path; *ptr != '\0'; ptr++) {
		if (*ptr == '/') file_name_start = ptr + 1;
	}

	return file_name_start;
}

void print_current_directory(const char* current_directory) {
	//for (unsigned int i = 0; i < 132; i++)
	//	LCD_BUFFER[i] = 0;


	unsigned int length = strlen(current_directory);

	uint8_t page = 0;
	uint8_t col = 0;

	const uint8_t* folder_icon = &FILE_EXPLORER_ICONS[0];
	uint8_t icon_length = 9;
	RP_DRAW_ICON(&col, 1, 1, &page, &col);

	RP_DRAW_ICON(folder_icon, icon_length, 1, &page, &col);
	RP_PRINT_TEXT(" ", &page, &col);

	if (length > 21) {
		RP_PRINT_TEXT("...", &page, &col);
		RP_PRINT_TEXT(current_directory + length - 18, &page, &col);
	} else {
		RP_PRINT_TEXT(current_directory, &page, &col);
	}
}

void print_file(const char* name, uint8_t start_page, bool is_selection, uint8_t list_indicator) {
	uint8_t page = start_page;
	uint8_t col = 0;


	RP_PRINT_TEXT(" ", &page, &col);
	uint8_t* list_icon = &FILE_EXPLORER_ICONS[FILE_EXPLORER_INDICES[list_indicator]];
	uint8_t icon_length = FILE_EXPLORER_INDICES[list_indicator + 1] - FILE_EXPLORER_INDICES[list_indicator];
	RP_DRAW_ICON(list_icon, icon_length, 1, &page, &col);

	RP_PRINT_TEXT(" ", &page, &col);

	if (is_selection) {
		RP_DRAW_ICON(&FILE_EXPLORER_ICONS[FILE_EXPLORER_INDICES[10]], FILE_EXPLORER_INDICES[11] - FILE_EXPLORER_INDICES[10], 1, &page, &col);
	} else	RP_PRINT_TEXT(" ", &page, &col);

	RP_PRINT_TEXT(name, &page, &col);
}

void render_current_file_list(const char* file1, const char* file2, bool first_selected, bool is_top, bool is_bottom) {
	uint8_t page = 1;
	uint8_t col = 0;
	if (file1[0] == '\0' && file2[0] == '\0') {
		RP_PRINT_TEXT("   EMPTY", &page, &col);
		return;
	}

	print_file(file1, 1, first_selected, (is_top && (file2[0] != '\0')) ? 7 : 8);

	if (file2[0] == '\0') return;

	print_file(file2, 2, !first_selected, (is_bottom) ? 9 : 8);
}

void render_file_selector(const char* current_directory, const char* file1, const char* file2, bool is_selected, bool is_top, bool is_bottom) {
	system_call(0x0013, 0);
	print_current_directory(current_directory);

	render_current_file_list(file1, file2, is_selected, is_top, is_bottom);

	for (int i = 0; i < 6; i++) {
		draw_file_explorer_icon(i + 1, 3, i * 22);
	}
}

bool is_correct_file_type(const char* file_path, const char* file_type) {
	unsigned int file_path_length = strlen(file_path);
	unsigned int file_type_length = strlen(file_type);

	if (file_path_length < file_type_length) return false;

	for (unsigned int i = 0; i < file_type_length; i++) {
		if (file_path[file_path_length - file_type_length + i] != file_type[i]) return false;
	}

	return true;
}

void _get_abbreviated_name(FILINFO* info, char buf[24]) {
	unsigned int length = strlen(info->fname);

	memset(buf, 0, 24);
	if (length < 22) memcpy(buf, info->fname, length + 1);
	else {
		memcpy(buf, "...", 3);
		memcpy(buf + 3, info->fname + length - 19, 19);
	}
}

void run_file_selector(const char* base, const char* file_type, char* result, unsigned int size_of_result) {
	unsigned int current_index = 0;

	if (base == NULL) {
		base = "/";
	}

	f_chdir(base);

	bool first_selected = true;
	bool last_up = false;

	char file0[24];
	char file1[24];
	char file2[24];

	char* file0ptr = file0;
	char* file1ptr = file1;
	char* file2ptr = file2;

	DIR dir;
	FILINFO f;

	f_opendir(&dir, ".");
	//RP_OPENDIR(base);

	f_readdir(&dir, &f);
	_get_abbreviated_name(&f, file0);
	f_readdir(&dir, &f);
	_get_abbreviated_name(&f, file1);
	f_readdir(&dir, &f);
	_get_abbreviated_name(&f, file2);

	//RP_NEXTFILE(files[0], &attributes);
	//RP_NEXTFILE(files[1], &attributes);

	//RP_CLOSEDIR();

	char* current_location = result;
	f_getcwd(current_location, size_of_result);
	render_file_selector(current_location, file0, file1, true, true, file2[0] == '\0');

	while (1) {
		//render_file_selector(current_location, files, total_files, selected_file);

		uint8_t key = system_call(2, 0);;
		if (key > 6) continue;

		switch (key) {
		case 3: // go back
			last_up = false;
			first_selected = true;
			current_index = 0;
			f_closedir(&dir);

			f_chdir("..");

			f_getcwd(current_location, size_of_result);
			f_opendir(&dir, ".");

			f_readdir(&dir, &f);
			_get_abbreviated_name(&f, file0);

			if (file0[0] != '\0') {
				f_readdir(&dir, &f);
				_get_abbreviated_name(&f, file1);

				if (file1[0] != '\0') {
					f_readdir(&dir, &f);
					_get_abbreviated_name(&f, file2);
				} else file2[0] = '\0';
			} else {
				file1[0] = '\0';
				file2[0] = '\0';
			}

			file0ptr = file0;
			file1ptr = file1;
			file2ptr = file2;


			render_file_selector(current_location, file0, file1, true, true, file2ptr[0] == '\0');

			break;
		case 4:
			// enter directory if available
			char* file2use = file1ptr;
			if (first_selected) file2use = file0ptr;
				if (f_chdir(file2use) == FR_OK) {
					current_index = 0;
					first_selected = true;
					last_up = true;

					f_closedir(&dir);
					f_opendir(&dir, ".");

					f_readdir(&dir, &f);
					_get_abbreviated_name(&f, file0);

					if (file0[0] != '\0') {
						f_readdir(&dir, &f);
						_get_abbreviated_name(&f, file1);

						if (file1[0] != '\0') {
							f_readdir(&dir, &f);
							_get_abbreviated_name(&f, file2);
						} else file2[0] = '\0';
					} else {
						file1[0] = '\0';
						file2[0] = '\0';
					}

					file0ptr = file0;
					file1ptr = file1;
					file2ptr = file2;

				f_getcwd(current_location, size_of_result);
				render_file_selector(current_location, file0ptr, file1ptr, true, true, file2ptr[0] == '\0');
			}

			break;
		case 1:
			// go up in list
			if (current_index == 0) break;
			current_index--;

			if (!first_selected) {
				first_selected = true;

				render_file_selector(current_location, file0ptr, file1ptr, true, ((!first_selected && current_index == 1) || current_index == 0), file2ptr[0] == '\0');
				break;
			}

			char* temp = file2ptr;
			file2ptr = file1ptr;
			file1ptr = file0ptr;
			file0ptr = temp;

			f_closedir(&dir);
			f_opendir(&dir, ".");

			for (int i = 0; i <= current_index; i++)
				f_readdir(&dir, &f);
			_get_abbreviated_name(&f, file0ptr);

			render_file_selector(current_location, file0ptr, file1ptr, true, ((!first_selected && current_index == 1) || current_index == 0), file2ptr[0] == '\0');
			last_up = true;
			break;
		case 2:
			// go down in list
			current_index++;

			if (first_selected) // pointer at top, so just shift the pointer
			{
				if (file1ptr[0] == '\0') {
					current_index--;
					break;
				}

				first_selected = false;

				render_file_selector(current_location, file0ptr, file1ptr, false, ((!first_selected && current_index == 1) || current_index == 0), file2ptr[0] == '\0');
				break;
			}

			if (file2ptr[0] == '\0') {
				current_index--;
				break;
			}

			temp = file2ptr;
			file2ptr = file0ptr;
			file0ptr = file1ptr;
			file1ptr = temp;

			f_readdir(&dir, &f);
			if (last_up) {
				for (int i = 0; i < 2; i++)
					f_readdir(&dir, &f);
				last_up = false;
			}
			_get_abbreviated_name(&f, file2ptr);

			render_file_selector(current_location, file0ptr, file1ptr, false, ((!first_selected && current_index == 1) || current_index == 0), file2ptr[0] == '\0');
			break;
		case 5:
			if (first_selected) {
				strcpy(result, file0ptr);
			} else strcpy(result, file1ptr);
			return;
		case 6:
			last_up = false;
			first_selected = true;
			current_index = 0;
			f_closedir(&dir);

			f_chdir(base);

			f_getcwd(current_location, size_of_result);
			f_opendir(&dir, ".");

			f_readdir(&dir, &f);
			_get_abbreviated_name(&f, file0);

			if (file0[0] != '\0') {
				f_readdir(&dir, &f);
				_get_abbreviated_name(&f, file1);

				if (file1[0] != '\0') {
					f_readdir(&dir, &f);
					_get_abbreviated_name(&f, file2);
				} else file2[0] = '\0';
			} else {
				file1[0] = '\0';
				file2[0] = '\0';
			}

			file0ptr = file0;
			file1ptr = file1;
			file2ptr = file2;

			render_file_selector(current_location, file0, file1, true, true, file2ptr[0] == '\0');
			break;
		}
	}
}
