// fetch_starlink.cpp
//
// Pulls the Starlink orbital-element feed from CelesTrak (OMM, JSON) over HTTPS
// and saves it locally, pretty-printed so it's readable. Native Windows fetch
// via WinHTTP -- no external libraries to install.
//
// Source feed (guide section 3.1):
//   https://celestrak.org/NORAD/elements/gp.php?GROUP=starlink&FORMAT=json
//
// Note: CelesTrak transmits minified (one-line) JSON to save bandwidth, which
// is the correct machine-to-machine format. Whitespace is insignificant in
// JSON, so we re-indent it locally purely for human reading.
//
// Build (run build.bat, or directly in a Developer prompt):
//   cl /std:c++latest /EHsc fetch_starlink.cpp
//
// Usage:
//   fetch_starlink.exe                 fetch "starlink", pretty-print, save
//   fetch_starlink.exe fetch oneweb    fetch any CelesTrak GROUP
//   fetch_starlink.exe format in.json [out.json]   re-indent a cached file
//                                                  (offline, no network)

#include <windows.h>
#include <winhttp.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <print>
#include <sstream>
#include <string>

#pragma comment(lib, "winhttp.lib")

namespace {

constexpr wchar_t kHost[] = L"celestrak.org";

// GET the given path from kHost over HTTPS and return the response body.
// Throws std::runtime_error on any failure.
std::string https_get(const std::wstring& path) {
  HINTERNET session = WinHttpOpen(L"leo-sim-fetch/0.1",
                                  WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY,
                                  WINHTTP_NO_PROXY_NAME,
                                  WINHTTP_NO_PROXY_BYPASS, 0);
  if (!session) throw std::runtime_error("WinHttpOpen failed");

  HINTERNET connect =
      WinHttpConnect(session, kHost, INTERNET_DEFAULT_HTTPS_PORT, 0);
  if (!connect) {
    WinHttpCloseHandle(session);
    throw std::runtime_error("WinHttpConnect failed");
  }

  HINTERNET request = WinHttpOpenRequest(
      connect, L"GET", path.c_str(), nullptr, WINHTTP_NO_REFERER,
      WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE);
  if (!request) {
    WinHttpCloseHandle(connect);
    WinHttpCloseHandle(session);
    throw std::runtime_error("WinHttpOpenRequest failed");
  }

  std::string body;
  bool ok = WinHttpSendRequest(request, WINHTTP_NO_ADDITIONAL_HEADERS, 0,
                               WINHTTP_NO_REQUEST_DATA, 0, 0, 0) &&
            WinHttpReceiveResponse(request, nullptr);

  // Check the HTTP status code (CelesTrak returns 403 if you poll too soon).
  DWORD status = 0;
  DWORD status_size = sizeof(status);
  if (ok) {
    WinHttpQueryHeaders(
        request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
        WINHTTP_HEADER_NAME_BY_INDEX, &status, &status_size,
        WINHTTP_NO_HEADER_INDEX);
  }

  if (ok && status == 200) {
    DWORD available = 0;
    while (WinHttpQueryDataAvailable(request, &available) && available > 0) {
      std::string chunk(available, '\0');
      DWORD read = 0;
      if (!WinHttpReadData(request, chunk.data(), available, &read)) break;
      chunk.resize(read);
      body += chunk;
    }
  }

  WinHttpCloseHandle(request);
  WinHttpCloseHandle(connect);
  WinHttpCloseHandle(session);

  if (!ok) throw std::runtime_error("HTTPS request failed");
  if (status != 200) {
    throw std::runtime_error("HTTP status " + std::to_string(status) +
                             " (403 means CelesTrak was polled too soon)");
  }
  return body;
}

// Re-indent JSON with two-space indentation. Walks the text character by
// character, tracking string state so braces/commas inside string values are
// left alone. Whitespace between tokens is insignificant, so we drop the
// source's and emit our own.
std::string pretty_print(const std::string& in) {
  std::string out;
  out.reserve(in.size() * 2);
  int indent = 0;
  bool in_string = false;
  bool escaped = false;

  auto newline = [&]() {
    out.push_back('\n');
    out.append(static_cast<std::size_t>(indent) * 2, ' ');
  };

  for (std::size_t i = 0; i < in.size(); ++i) {
    const char c = in[i];

    if (in_string) {
      out.push_back(c);
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        in_string = false;
      }
      continue;
    }

    switch (c) {
      case '"':
        in_string = true;
        out.push_back(c);
        break;
      case '{':
      case '[': {
        // Keep empty containers on one line: {} or [].
        std::size_t j = i + 1;
        while (j < in.size() &&
               std::isspace(static_cast<unsigned char>(in[j]))) {
          ++j;
        }
        const char close = (c == '{') ? '}' : ']';
        if (j < in.size() && in[j] == close) {
          out.push_back(c);
          out.push_back(close);
          i = j;
        } else {
          out.push_back(c);
          ++indent;
          newline();
        }
        break;
      }
      case '}':
      case ']':
        --indent;
        newline();
        out.push_back(c);
        break;
      case ',':
        out.push_back(c);
        newline();
        break;
      case ':':
        out.append(": ");
        break;
      case ' ':
      case '\t':
      case '\n':
      case '\r':
        break;  // drop insignificant whitespace
      default:
        out.push_back(c);
    }
  }
  out.push_back('\n');
  return out;
}

std::string read_file(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open " + path.string());
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void write_file(const std::filesystem::path& path, const std::string& data) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream(path, std::ios::binary)
      .write(data.data(), static_cast<std::streamsize>(data.size()));
}

// Rough record count: one object per satellite => count '{' braces.
std::size_t count_records(const std::string& json) {
  return static_cast<std::size_t>(std::count(json.begin(), json.end(), '{'));
}

int do_fetch(const std::string& group) {
  const std::wstring wgroup(group.begin(), group.end());
  const std::wstring path =
      L"/NORAD/elements/gp.php?GROUP=" + wgroup + L"&FORMAT=json";

  std::println("Fetching '{}' elements from celestrak.org ...", group);

  std::string json;
  try {
    json = https_get(path);
  } catch (const std::exception& e) {
    std::println(stderr, "Fetch failed: {}", e.what());
    return 1;
  }

  if (json.size() < 4 || json.front() != '[') {
    std::println(stderr, "Unexpected response ({} bytes): {}", json.size(),
                 json.substr(0, 200));
    return 1;
  }

  const std::string pretty = pretty_print(json);
  const std::filesystem::path out = "data/" + group + ".json";
  write_file(out, pretty);

  std::println("Saved {} satellites to {} ({} bytes, indented).",
               count_records(json), out.string(), pretty.size());
  std::println("\n--- preview (first record) ---");
  // First record spans from the opening '{' to its matching '}'.
  const std::size_t start = pretty.find('{');
  const std::size_t end = pretty.find("\n  }");
  std::println("{}", pretty.substr(start, end == std::string::npos
                                              ? 400
                                              : end - start + 4));
  return 0;
}

int do_format(const std::filesystem::path& infile,
              const std::filesystem::path& outfile) {
  std::string json;
  try {
    json = read_file(infile);
  } catch (const std::exception& e) {
    std::println(stderr, "{}", e.what());
    return 1;
  }
  const std::string pretty = pretty_print(json);
  write_file(outfile, pretty);
  std::println("Re-indented {} satellites: {} -> {} ({} bytes).",
               count_records(json), infile.string(), outfile.string(),
               pretty.size());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string cmd = (argc > 1) ? argv[1] : "fetch";

  if (cmd == "format") {
    if (argc < 3) {
      std::println(stderr, "usage: fetch_starlink format <in.json> [out.json]");
      return 2;
    }
    const std::filesystem::path infile = argv[2];
    const std::filesystem::path outfile = (argc > 3) ? argv[3] : argv[2];
    return do_format(infile, outfile);
  }

  // Default + "fetch": optional group argument.
  std::string group = "starlink";
  if (cmd == "fetch") {
    if (argc > 2) group = argv[2];
  } else {
    group = cmd;  // e.g. `fetch_starlink oneweb`
  }
  return do_fetch(group);
}
