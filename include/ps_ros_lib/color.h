/**
 * @file color.h
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef COLOR_HPP
#define COLOR_HPP

#include "ros/ros.h"

namespace color
{
    // A map of strings to strings. The first string is the name of the format, the second string is the code to reset that format.
    const std::map<std::string, std::string> color_foreground = {
        {"Default", "39"},
        {"Black", "30"},
        {"Red", "31"},
        {"Green", "32"},
        {"Yellow", "33"},
        {"Blue", "34"},
        {"Magenta", "35"},
        {"Cyan", "36"},
        {"Light Gray", "37"},
        {"Dark Gray", "90"},
        {"Light Red", "91"},
        {"Light Green", "92"},
        {"Light Yellow", "93"},
        {"Light Blue", "94"},
        {"Light Magenta", "95"},
        {"Light Cyan", "96"},
        {"White", "97"}};

    // A map of strings to strings. The first string is the name of the format, the second string is the code to reset that format.
    const std::map<std::string, std::string> color_background = {
        {"Default", "49"},
        {"Black", "40"},
        {"Red", "41"},
        {"Green", "42"},
        {"Yellow", "43"},
        {"Blue", "44"},
        {"Magenta", "45"},
        {"Cyan", "46"},
        {"Light Gray", "47"},
        {"Dark Gray", "100"},
        {"Light Red", "101"},
        {"Light Green", "102"},
        {"Light Yellow", "103"},
        {"Light Blue", "104"},
        {"Light Magenta", "105"},
        {"Light Cyan", "106"},
        {"White", "107"}};

    // A map of strings to strings. The first string is the name of the format, the second string is the code to reset that format.
    const std::map<std::string, std::string> format_set = {
        {"Default", "0"},
        {"Bold", "1"},
        {"Dim", "2"},
        {"Underlined", "4"},
        {"Blink", "5"},
        {"Reverse", "7"},
        {"Hidden", "8"}};

    // A map of strings to strings. The first string is the name of the format, the second string is the code to reset that format.
    const std::map<std::string, std::string> format_reset = {
        {"Default", "0"},
        {"Bold", "21"},
        {"Dim", "22"},
        {"Underlined", "24"},
        {"Blink", "25"},
        {"Reverse", "27"},
        {"Hidden", "28"}};

    /**
     * @brief This function will return a string with the control code to change the color of the text.
     *
     * @param _text Text to be colored.
     * @param _color_foreground Color of the foreground.
     * @param _color_background Color of the background.
     * @param _format_set Format to be set to.
     * @param _format_reset Format to be reset to.
     * @return std::string String with the control code to change the color of the text.
     */
    std::string rize(std::string _text,
                     std::string _color_foreground = "Default",
                     std::string _color_background = "Default",
                     std::string _format_set = "Default",
                     std::string _format_reset = "Default")
    {
        // Checking if the input is valid. If not, it will set it to default.
        if (color_foreground.find(_color_foreground) == color_foreground.end())
            _color_foreground = "Default";
        if (color_background.find(_color_background) == color_background.end())
            _color_background = "Default";
        if (format_set.find(_format_set) == format_set.end())
            _format_set = "Default";
        if (format_reset.find(_format_reset) == format_reset.end())
            _format_reset = "Default";

        // Setting the control code to the escape character.
        std::string control_code = "\033[";

        // Concatenating the control code with the format set, foreground, background, and text. Then it concatenates the control code with the format reset.
        return control_code + format_set.at(_format_set) + ";" + color_foreground.at(_color_foreground) + ";" + color_background.at(_color_background) + "m" +
               _text +
               control_code + format_reset.at(_format_reset) + "m";
    }

    /**
     * @brief This function will return a string with the control code to change the color of the text.
     *
     * @param _text Text to be colored.
     * @param _x Position of the text in the x-axis.
     * @param _y Position of the text in the y-axis.
     * @param _color_foreground Color of the foreground.
     * @param _color_background Color of the background.
     * @param _format_set Format to be set to.
     * @param _format_reset Format to be reset to.
     * @return std::string String with the control code to change the color of the text.
     */
    std::string rize(std::string _text,
                     unsigned int _x,
                     unsigned int _y,
                     std::string _color_foreground = "Default",
                     std::string _color_background = "Default",
                     std::string _format_set = "Default",
                     std::string _format_reset = "Default")
    {
        // Checking if the input is valid. If not, it will set it to default.
        if (color_foreground.find(_color_foreground) == color_foreground.end())
            _color_foreground = "Default";
        if (color_background.find(_color_background) == color_background.end())
            _color_background = "Default";
        if (format_set.find(_format_set) == format_set.end())
            _format_set = "Default";
        if (format_reset.find(_format_reset) == format_reset.end())
            _format_reset = "Default";

        // Setting the control code to the escape character.
        std::string control_code = "\033[";

        // Concatenating the control code with the format set, foreground, background, and text. Then it concatenates the control code with the format reset.
        return control_code + std::to_string(_y) + ";" + std::to_string(_x) + "H" +
               control_code + format_set.at(_format_set) + ";" + color_foreground.at(_color_foreground) + ";" + color_background.at(_color_background) + "m" +
               _text +
               control_code + format_reset.at(_format_reset) + "m";
    }
}

#endif