#ifndef AUX_METHODS_HPP
#define AUX_METHODS_HPP

#include <logger.hpp>
#include <string>
#include <sstream>




template <typename T> std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}



/*
 * PickRandomElementFromArray(): pick uniformely random an element of an array
 */
int PickRandomElementFromArray(int *array, int array_size);

/*
 * PrintOrWriteArrayInt(): prints per console or writes to a given file the elements of an "int" array
 */
void PrintOrWriteArrayInt(int *list, int list_size, int write_or_print, int save_node_logs,
		int print_node_logs, Logger node_logger);

/*
 * PrintOrWriteArrayDouble(): prints per console or writes to a given file the elements of an "double" array
 */
void PrintOrWriteArrayDouble(double *list, int list_size, int write_or_print, int save_node_logs,
		int print_node_logs, Logger node_logger);

/*
 * GetFirstOrLastTrueElemOfArray(): pick the first or last TRUE element of an array
 */
int GetFirstOrLastTrueElemOfArray(int first_or_last, int *list, int list_size);

int GetNumberOfSpecificElementInArray(int value, int* array, int list_size);

double RandomDouble(double min, double max);

double truncate_Sergio(double number, int floating_position);

double round_to_digits(double value, int digits);

double round_to_digits_float(float value, int digits);

double fix_time_offset(double time_value, int trunc_pos, int round_pos);

#endif
