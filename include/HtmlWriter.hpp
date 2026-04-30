#pragma once

#include <string>

class HtmlWriter {
public:
    [[nodiscard]] static std::string wrap(const std::string& json);
};