//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "common/configurator.h"

#include <lua.hpp>
#include <assert.h>

#include <iostream>
#include <fstream>
#include <streambuf>
#include <algorithm>
#include <functional>
#include <cmath>

namespace {

void SaveExtractor(lua_State *L, Configurator *extractor)
{
    lua_pushstring(L, "this");
    lua_pushlightuserdata(L, extractor);
    lua_settable(L, LUA_REGISTRYINDEX);
}

Configurator *LoadExtractor(lua_State *L)
{
    lua_pushstring(L, "this");
    lua_gettable(L, LUA_REGISTRYINDEX);
    void *value = lua_isnil(L, -1) ? nullptr : lua_touserdata(L, -1);
    lua_pop(L, 1);

    if (value == nullptr)
        return nullptr;

    return reinterpret_cast<Configurator *>(value);
}

void SetValue(lua_State *L, const int key) { lua_pushinteger(L, key); }
void SetValue(lua_State *L, const std::string &key) { lua_pushstring(L, key.c_str()); }

template <typename T>
void GetValue(lua_State *L, const T &key)
{
    SetValue(L, key);
    lua_rawget(L, -2);
}

template <typename T>
bool HasKeyOfType(lua_State *L, const T &key)
{
    if (!lua_istable(L, -1)) {
        std::cerr << "Table is not at the top of the stack" << std::endl;
        return false;
    }

    SetValue(L, key);
    lua_rawget(L, -2);
    const bool key_not_found = lua_isnil(L, -1);
    lua_pop(L, 1);  // Pop the item again.
    return !key_not_found;
}

void GetArray(lua_State *L, const std::function<void()> &pop_value)
{
    int index = 1;
    while (true) {
        GetValue(L, index);
        if (lua_isnil(L, -1)) {
            lua_pop(L, 1);
            break;
        }

        pop_value();
        ++index;
    }
}

}  // namespace

std::unique_ptr<Configurator> Configurator::Create(const std::vector<std::string> &directories, const std::string &basename)
{
    if (directories.empty()) {
        std::cerr << "Invalid lua script directories" << std::endl;
        return nullptr;
    }

    if (basename.empty()) {
        std::cerr << "Invalid lua file name" << std::endl;
        return nullptr;
    }

    std::unique_ptr<ILoader> resolver(new Loader(directories));
    if (resolver == nullptr) {
        std::cerr << "Failed to create File Resolver" << std::endl;
        return nullptr;
    }

    std::string code;
    if (!resolver->GetFileContent(basename, code)) {
        std::cerr << "Failed to load lua script" << std::endl;
        return nullptr;
    }

    // create lua vritual machine
    lua_State *L = luaL_newstate();
    if (L == nullptr) {
        std::cerr << "Failed to create lua virtual machine" << std::endl;
        return nullptr;
    }

    luaL_openlibs(L);

    std::unique_ptr<Configurator> extractor(new Configurator(L, std::move(resolver)));
    if (extractor == nullptr) {
        std::cerr << "Failed to create Parameter Extractor" << std::endl;
        return nullptr;
    }

    if (!extractor->Initialize(code)) {
        std::cerr << "Failed to initialize Configurator" << std::endl;
        return nullptr;
    }

    return extractor;
}

Configurator::Configurator(lua_State *L, std::shared_ptr<ILoader> file_resolver)
    : L_(L), ref_(-1), file_resolver_(std::move(file_resolver)), status_(Status::NO_ERROR) { }

Configurator::~Configurator()
{
    if (ref_ > 0) {
        luaL_unref(L_, LUA_REGISTRYINDEX, ref_);
    } else {
        lua_close(L_);
    }
}

bool Configurator::Initialize(const std::string &code)
{
    assert(L_ != nullptr);
    SaveExtractor(L_, this);

    lua_register(L_, "include", LoadLuaScript);

    if (luaL_loadstring(L_, code.c_str()) != LUA_OK) {
        std::cerr << "Failed to load lua script" << std::endl;
        return false;
    }

    if (lua_pcall(L_, 0, 1, 0) != LUA_OK) {
        std::cerr << "Failed to execute function" << std::endl;
        return false;
    }

    if (status_ != Status::NO_ERROR) {
        std::cerr << "Failed to load lua scripts" << std::endl;
        return false;
    }

    if (!lua_istable(L_, -1)) {
        std::cerr << "Lua script format error" << std::endl;
        return false;
    }

    return true;
}

bool Configurator::GetArray(std::vector<double> &values)
{
    bool isok = true;
    ::GetArray(L_, [&values, &isok, this] {
        if (!lua_isnumber(L_, -1)) {
            isok = false;
            return;
        }

        values.emplace_back(lua_tonumber(L_, -1));
        lua_pop(L_, 1);
    });

    return isok;
}

bool Configurator::GetArray(std::vector<std::unique_ptr<Configurator>> &values)
{
    bool isok = true;
    ::GetArray(L_, [&values, &isok, this] {
        std::unique_ptr<Configurator> extractor = AllocRef();
        if (extractor == nullptr) {
            isok = false;
            return;
        }

        values.emplace_back(std::move(extractor));
    });

    return isok;
}

std::vector<std::string> Configurator::GetKeys() const
{
    if (!lua_istable(L_, -1)) {
        std::cerr << "Table is not at the top of the stack" << std::endl;
        return std::vector<std::string>{};
    }

    std::vector<std::string> keys;

    lua_pushnil(L_);  // Push the first key
    while (lua_next(L_, -2) != 0) {
        lua_pop(L_, 1);  // Pop value, keep key.
        if (!lua_isnumber(L_, -1))
            keys.emplace_back(lua_tostring(L_, -1));
    }

    return keys;
}

bool Configurator::HasKey(const std::string &key) const
{
    if (!HasKeyOfType(L_, key)) {
        std::cerr << "Failed to found key: " << key << std::endl;
        return false;
    }

    return true;
}

bool Configurator::GetString(const std::string &key, std::string &value)
{
    if (!HasKey(key))
        return false;

    GetValue(L_, key);

    if (!lua_tostring(L_, -1)) {
        std::cerr << "The key " << key << " isn't string" << std::endl;
        return false;
    }

    value = lua_tostring(L_, -1);
    lua_pop(L_, 1);

    return true;
}

bool Configurator::GetDouble(const std::string &key, double &value)
{
    if (!HasKey(key))
        return false;

    GetValue(L_, key);

    if (!lua_isnumber(L_, -1)) {
        std::cerr << "The key " << key << " isn't number" << std::endl;
        return false;
    }

    value = lua_tonumber(L_, -1);
    lua_pop(L_, 1);

    return true;
}

bool Configurator::GetInt(const std::string &key, int &value)
{
    if (!HasKey(key))
        return false;

    GetValue(L_, key);

    if (!lua_isnumber(L_, -1)) {
        std::cerr << "The key " << key << " isn't number" << std::endl;
        return false;
    }

    value = lua_tointeger(L_, -1);
    lua_pop(L_, 1);

    return true;
}

bool Configurator::GetBool(const std::string &key, bool &value)
{
    if (!HasKey(key))
        return false;

    GetValue(L_, key);

    if (!lua_isboolean(L_, -1)) {
        std::cerr << "The key " << key << " isn't boolean" << std::endl;
        return false;
    }

    value = lua_toboolean(L_, -1);
    lua_pop(L_, 1);
    return true;
}

std::unique_ptr<Configurator> Configurator::GetExtractor(const std::string &key)
{
    if (!HasKey(key))
        return nullptr;

    GetValue(L_, key);
    return AllocRef();
}

std::unique_ptr<Configurator> Configurator::AllocRef()
{
    if (!lua_istable(L_, -1)) {
        std::cerr << "Table is not at the top of the stack" << std::endl;
        return nullptr;
    }

    lua_State *L = lua_newthread(L_);
    if (L == nullptr) {
        std::cerr << "Failed to create lua thread" << std::endl;
        return nullptr;
    }

    if (!lua_isthread(L_, -1)) {
        std::cerr << "Lua parent isn't thread" << std::endl;
        return nullptr;
    }

    int ref = luaL_ref(L_, LUA_REGISTRYINDEX);

    if (!lua_istable(L_, -1)) {
        std::cerr << "Lua script format error" << std::endl;
        return nullptr;
    }

    // Moves the table and the coroutine over
    lua_xmove(L_, L, 1);

    if (!lua_istable(L, -1)) {
        std::cerr << "Table is not at the top of the stack" << std::endl;
        return nullptr;
    }

    std::unique_ptr<Configurator> extractor(new Configurator(L, file_resolver_));
    if (extractor == nullptr) {
        std::cerr << "Failed to create Configurator" << std::endl;
        return nullptr;
    }

    extractor->ref_ = ref;
    return extractor;
}

int Configurator::LoadLuaScript(lua_State *L)
{
    assert(lua_gettop(L) == 1);
    assert(lua_isstring(L, -1));

    Configurator *extractor = LoadExtractor(L);
    assert(extractor != nullptr);

    const std::string basename = lua_tostring(L, -1);
    std::string filename;
    if (!extractor->file_resolver_->GetFullPath(basename, filename)) {
        extractor->status_ |= Status::FILENAME_GET_FAILED;
        return lua_gettop(L);
    }

    auto &files = extractor->included_files_;
    if (std::find(files.begin(), files.end(), filename) != files.end()) {
        extractor->status_ |= Status::SCRIPT_INCLUDE_TWICE;
        return lua_gettop(L);
    }

    files.emplace_back(filename);
    lua_pop(L, 1);

    assert(lua_gettop(L) == 0);
    std::string content;
    if (!extractor->file_resolver_->GetFileContent(basename, content)) {
        extractor->status_ |= Status::CONTENT_GET_FAILED;
        return lua_gettop(L);
    }

    if (luaL_loadbuffer(L, content.c_str(), content.size(), filename.c_str()) != LUA_OK) {
        extractor->status_ |= Status::LUA_LOAD_BUFFER_FAILED;
        return lua_gettop(L);
    }

    if (lua_pcall(L, 0, LUA_MULTRET, 0) != LUA_OK) {
        extractor->status_ |= Status::LUA_PCALL_FAILED;
        return lua_gettop(L);
    }

    return lua_gettop(L);
}
