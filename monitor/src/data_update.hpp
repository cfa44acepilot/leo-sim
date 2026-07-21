/*****************************************************************************
  filename data_update.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    Background refresh of the ON-DISK OMM snapshot, behind the monitor's
    "Update data" button. Shells out to the external fetch tool exactly as
    `leo update` does, on a worker thread, so a multi-second network call never
    stalls the render loop.

    This writes the FILE and nothing else. It deliberately does NOT reload,
    re-propagate, or rebuild anything in the running Simulator: a reload is
    Tier 1 (load + build_topology), which reorders every dense array and
    renumbers every NodeId out from under the selection, the route, and the
    highlight. The running scene therefore keeps showing the OLD data, and the
    UI says so in as many words.
 *****************************************************************************/

#ifndef MONITOR_DATA_UPDATE_HPP
#define MONITOR_DATA_UPDATE_HPP

#include <filesystem>  /* fs::path: the snapshot and fetch-tool locations   */
#include <future>      /* std::future: the fetch result, polled not waited  */
#include <string>      /* the user-facing status message                    */

namespace monitor
{

/* What the worker thread hands back: the verdict, plus the sentence to show
   the user -- the message is built on the worker because only it knows why a
   fetch failed. */
struct FetchOutcome
{
  bool ok = false;         /* false unless the tool ran AND exited zero      */
  std::string message;     /* shown verbatim in the Simulation panel         */
};

/* One in-flight fetch at a time. Driven from the render thread only: start()
   on the button press, poll() once per frame. */
class DataUpdater
{
 public:
  enum class State
  {
    kIdle,
    kFetching,
    kDone,
    kFailed
  };

  /*---------------------------------------------------------------------------
    Function: start
    Description: Spawn the fetch on a background thread. Ignored while one is
                 already in flight, so a double-click cannot launch two
                 downloads over each other.
    Input: omm     -- the snapshot file the fetch tool will overwrite
           fetcher -- path to the external fetch tool to spawn
    Outputs: None; moves the updater into kFetching.
  ---------------------------------------------------------------------------*/
  void start(
      const std::filesystem::path& omm,      /* file being refreshed        */
      const std::filesystem::path& fetcher   /* tool that does the network  */
  );

  /*---------------------------------------------------------------------------
    Function: poll
    Description: Promote kFetching to kDone/kFailed once the worker returns.
                 Non-blocking by construction, so the render thread can call it
                 every frame without ever waiting on the network.
    Input: None.
    Outputs: None; does nothing when idle or already finished.
  ---------------------------------------------------------------------------*/
  void poll();

  State state() const { return state_; }
  bool busy() const { return state_ == State::kFetching; }
  const std::string& message() const { return message_; }

 private:
  std::future<FetchOutcome> future_;  /* valid only while a fetch is running */
  State state_ = State::kIdle;        /* drives both the button and the text */
  std::string message_;               /* last thing worth telling the user   */
};

}  /* namespace monitor */

#endif  /* MONITOR_DATA_UPDATE_HPP */
