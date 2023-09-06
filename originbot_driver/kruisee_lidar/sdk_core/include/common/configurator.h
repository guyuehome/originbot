//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "common/loader.h"

typedef struct lua_State lua_State;

class Configurator {
public:
    static std::unique_ptr<Configurator> Create(const std::vector<std::string> &directories, const std::string &basename);

    Configurator(const Configurator &) = delete;
    Configurator &operator=(const Configurator &) = delete;

    ~Configurator();

    std::unique_ptr<Configurator> GetExtractor(const std::string &key);

    std::vector<std::string> GetKeys() const;
    bool HasKey(const std::string &key) const;

    bool GetString(const std::string &key, std::string &value);
    bool GetDouble(const std::string &key, double &value);
    bool GetInt(const std::string &key, int &value);
    bool GetBool(const std::string &key, bool &value);

    bool GetArray(std::vector<double> &values);
    bool GetArray(std::vector<std::unique_ptr<Configurator>> &extractors);

private:
    Configurator(lua_State *L, std::shared_ptr<ILoader> file_resolver);

    std::unique_ptr<Configurator> AllocRef();

    bool Initialize(const std::string &code);

    static int LoadLuaScript(lua_State* L);

private:
    lua_State *L_;
    int ref_;

    // This is shared with all the sub dictionaries.
    const std::shared_ptr<ILoader> file_resolver_;

    // List of all included files in order of inclusion. Used to prevent double inclusion.
    std::vector<std::string> included_files_;

    enum Status : uint32_t {
        NO_ERROR = 0,
        STACK_CHECK_FAILED = (1<<0),
        PARAMERATER_TYPE_CHECK_FAILED = (1<<2),
        FILENAME_GET_FAILED = (1<<3),
        SCRIPT_INCLUDE_TWICE = (1<<4),
        CONTENT_GET_FAILED = (1<<5),
        LUA_LOAD_BUFFER_FAILED = (1<<6),
        LUA_PCALL_FAILED = (1<<7)
    };

    uint32_t status_;
};