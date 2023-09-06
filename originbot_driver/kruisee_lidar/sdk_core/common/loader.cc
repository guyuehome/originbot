//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "common/loader.h"

#include <fstream>
#include <streambuf>

Loader::Loader(const std::vector<std::string> &directories)
    : directories_(directories) { }

bool Loader::GetFullPath(const std::string &basename, std::string &filename)
{
    for (const auto & path : directories_) {
        filename = path + "/" + basename;
        std::ifstream stream(filename.c_str());
        if (stream.good())
            return true;
    }

    return false;
}

bool Loader::GetFileContent(const std::string &basename, std::string &context)
{
    std::string filename;
    if (!GetFullPath(basename, filename))
        return false;

    std::ifstream stream(filename.c_str());
    if (!stream.good())
        return false;

    context = std::move(std::string(std::istreambuf_iterator<char>{stream}, std::istreambuf_iterator<char>{}));
    return true;
}