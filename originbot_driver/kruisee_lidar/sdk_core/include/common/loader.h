//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <string>
#include <vector>

class ILoader {
public:
    virtual ~ILoader() = default;
    virtual bool GetFullPath(const std::string &basename, std::string &filename) = 0;
    virtual bool GetFileContent(const std::string &basename, std::string &context) = 0;
};

class Loader : public ILoader {
public:
    explicit Loader(const std::vector<std::string> &directories);

    bool GetFullPath(const std::string &basename, std::string &filename);
    bool GetFileContent(const std::string &basename, std::string &context);

private:
    std::vector<std::string> directories_;
};