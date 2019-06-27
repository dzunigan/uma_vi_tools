// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#ifndef STRING_HPP_
#define STRING_HPP_

// STL
#include <algorithm>
#include <cstdarg>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// Boost
#include <boost/algorithm/string.hpp>

namespace {

inline void StringAppendV(std::string* dst, const char* format, va_list ap) {
    // First try with a small fixed size buffer.
    static const int kFixedBufferSize = 1024;
    char fixed_buffer[kFixedBufferSize];

    // It is possible for methods that use a va_list to invalidate
    // the data in it upon use.  The fix is to make a copy
    // of the structure before using it and use that copy instead.
    va_list backup_ap;
    va_copy(backup_ap, ap);
    int result = vsnprintf(fixed_buffer, kFixedBufferSize, format, backup_ap);
    va_end(backup_ap);

    if (result < kFixedBufferSize) {
      if (result >= 0) {
        // Normal case - everything fits.
        dst->append(fixed_buffer, result);
        return;
      }

      if (result < 0) {
        // Just an error.
        return;
      }
    }

    // Increase the buffer size to the size requested by vsnprintf,
    // plus one for the closing \0.
    const int variable_buffer_size = result + 1;
    std::unique_ptr<char> variable_buffer(new char[variable_buffer_size]);

    // Restore the va_list before we use it again.
    va_copy(backup_ap, ap);
    result =
        vsnprintf(variable_buffer.get(), variable_buffer_size, format, backup_ap);
    va_end(backup_ap);

    if (result >= 0 && result < variable_buffer_size) {
      dst->append(variable_buffer.get(), result);
    }
}

inline bool IsNotWhiteSpace(const int character) {
    return character != ' ' && character != '\n' && character != '\r' &&
            character != '\t';
}

}  // namespace

// Format string by replacing embedded format specifiers with their respective
// values, see `printf` for more details. This is a modified implementation
// of Google's BSD-licensed StringPrintf function.
inline std::string StringPrintf(const char* format, ...) {
    va_list ap;
    va_start(ap, format);
    std::string result;
    StringAppendV(&result, format, ap);
    va_end(ap);
    return result;
}

// Replace all occurrences of `old_str` with `new_str` in the given string.
inline std::string StringReplace(const std::string& str, const std::string& old_str,
                          const std::string& new_str) {
    if (old_str.empty()) {
      return str;
    }
    size_t position = 0;
    std::string mod_str = str;
    while ((position = mod_str.find(old_str, position)) != std::string::npos) {
      mod_str.replace(position, old_str.size(), new_str);
      position += new_str.size();
    }
    return mod_str;
}

// Split string into list of words using the given delimiters.
inline std::vector<std::string> StringSplit(const std::string& str,
                                     const std::string& delim) {
    std::vector<std::string> elems;
    boost::split(elems, str, boost::is_any_of(delim), boost::token_compress_on);
    return elems;
}

// Check whether a string starts with a certain prefix.
inline bool StringStartsWith(const std::string& str, const std::string& prefix) {
    return !prefix.empty() && prefix.size() <= str.size() &&
            str.substr(0, prefix.size()) == prefix;
}

// Remove whitespace from string on both, left, or right sides.
inline void StringLeftTrim(std::string* str) { str->erase(str->begin(), std::find_if(str->begin(), str->end(), IsNotWhiteSpace)); }
inline void StringRightTrim(std::string* str) { str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(), str->end()); }
inline void StringTrim(std::string* str) { StringLeftTrim(str); StringRightTrim(str); }

// Convert string to lower/upper case.
inline void StringToLower(std::string* str) { std::transform(str->begin(), str->end(), str->begin(), ::tolower); }
inline void StringToUpper(std::string* str) { std::transform(str->begin(), str->end(), str->begin(), ::toupper); }

// Check whether the sub-string is contained in the given string.
inline bool StringContains(const std::string& str, const std::string& sub_str) { return str.find(sub_str) != std::string::npos; }

#endif // STRING_HPP_
