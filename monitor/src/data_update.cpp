/*****************************************************************************
  filename data_update.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    Implementation of the background "Update data" fetch. See data_update.hpp
    for the contract and for why this touches the file on disk and nothing in
    the running simulator.
 *****************************************************************************/

#include "data_update.hpp"

#include <chrono>       /* zero-length wait_for: poll without blocking       */
#include <cstdio>       /* std::snprintf: format the new file's age          */
#include <cstdlib>      /* std::system: the shell-out `leo update` also uses */
#include <string>       /* std::to_string for the exit code                  */
#include <system_error> /* std::error_code: non-throwing filesystem queries  */

#include "cli.hpp" /* leo::omm_data_age_days: the CLI's freshness check */

namespace monitor
{
namespace
{

/*---------------------------------------------------------------------------
  Function: run_fetch
  Description: The fetch step of `leo update`, mirrored onto a worker thread.
               It cannot simply be CALLED: the CLI's run_update() lives in the
               `leo` executable, not in the constellation library the monitor
               links, so only the shell-out is duplicated -- the freshness
               measurement reuses leo::omm_data_age_days, which is why the two
               front ends can never disagree about what "fresh" means.

               The renderer still never speaks HTTP: it spawns the tool exactly
               as the CLI does, so the portable sim library stays network-free.
               A missing tool is reported, never fatal.

               Runs on the worker thread. std::system is not guaranteed
               thread-safe, but DataUpdater admits one in-flight fetch at a time
               and calls it nowhere else.
  Input: omm     -- snapshot path, re-measured after a successful fetch
         fetcher -- the external tool to spawn (by value: the worker outlives
                    the caller's locals)
  Outputs: The outcome, including the sentence shown in the UI.
---------------------------------------------------------------------------*/
FetchOutcome run_fetch(
    std::filesystem::path omm,    /* file the tool overwrites, then measured */
    std::filesystem::path fetcher /* made absolute below, hence by value     */
)
{
  FetchOutcome out;   /* built up as we learn how the fetch went           */
  std::error_code ec; /* exists()/absolute() must not throw on a junk path */

  /* Guard the missing tool BEFORE spawning a shell: std::system on a bad path
     yields an opaque non-zero exit that reads like a network failure. */
  if (!std::filesystem::exists(fetcher, ec))
  {
    out.message = "Fetch tool not found at '" + fetcher.string() +
                  "'. Build fetch_starlink.exe in the repo root, or pass "
                  "--fetcher <path>.";
    return out;
  }

  /* cmd.exe does not reliably search the current directory, so a relative path
     must be made absolute; the result needs quoting because the repo path can
     contain spaces ("D:\Leo Sim"). Same reasoning as the CLI's run_update. */
  if (fetcher.is_relative())
  {
    fetcher = std::filesystem::absolute(fetcher, ec);
  }
  const std::string cmd = /* quoted: the path has spaces */
      "\"" + fetcher.string() + "\" fetch starlink";
  const int rc = std::system(cmd.c_str()); /* the tool's own exit code    */
  if (rc != 0)
  {
    out.message = "Update failed (fetch tool exit " + std::to_string(rc) +
                  "). CelesTrak rate-limits repeat requests with HTTP 403 -- "
                  "wait a bit and retry.";
    return out;
  }

  out.ok = true;
  /* Re-measure the file we just wrote. This parses a multi-megabyte JSON, so it
     belongs on the worker -- doing it after the join would stutter a frame. */
  const double age = leo::omm_data_age_days(omm); /* < 0 if unreadable      */
  out.message = "Data updated";
  if (age >= 0.0)
  {
    char buf[48]; /* " (now N.N days old)" -- the age is small and bounded   */
    std::snprintf(buf, sizeof(buf), " (now %.1f days old)", age);
    out.message += buf;
  }
  /* The restart instruction is the whole point of the message: the scene on
     screen is still built from the snapshot loaded at startup. */
  out.message +=
      ". Restart the monitor to load it -- the scene on screen is still the "
      "OLD data.";
  return out;
}

} /* namespace */

/*---------------------------------------------------------------------------
  Function: DataUpdater::start
  Description: Launch the fetch. See the header.
  Input: omm, fetcher -- forwarded to run_fetch on the worker
  Outputs: None; state becomes kFetching and the message becomes provisional.
---------------------------------------------------------------------------*/
void DataUpdater::start(
    const std::filesystem::path& omm,    /* snapshot to refresh            */
    const std::filesystem::path& fetcher /* tool to spawn                  */
)
{
  if (state_ == State::kFetching)
  {
    return; /* one download at a time; a second click is simply ignored     */
  }
  state_ = State::kFetching;
  message_ = "fetching...";
  /* A std::async future BLOCKS in its destructor. That is what we want on exit:
     the app waits for an in-flight fetch rather than tearing the thread down
     mid-download and leaving a half-written starlink.json behind. */
  future_ = std::async(std::launch::async, run_fetch, omm, fetcher);
}

/*---------------------------------------------------------------------------
  Function: DataUpdater::poll
  Description: Collect the worker's result if it is ready. See the header.
  Input: None.
  Outputs: None; on completion the state and message become final.
---------------------------------------------------------------------------*/
void DataUpdater::poll()
{
  if (state_ != State::kFetching || !future_.valid())
  {
    return; /* nothing in flight -- the common case, called every frame     */
  }
  /* Zero-length wait: asks "is it done?" and returns immediately either way, so
     the render thread is never blocked on the network call. */
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return;
  }
  const FetchOutcome out = future_.get(); /* also releases the shared state */
  state_ = out.ok ? State::kDone : State::kFailed;
  message_ = out.message;
}

} /* namespace monitor */
