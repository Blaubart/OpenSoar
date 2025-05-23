// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "SkysightAPI.hpp"
#include "APIQueue.hpp"
#include "APIGlue.hpp"
#include "Request.hpp"

#include "Layers.hpp"
#include "ui/event/Timer.hpp"
#include "time/DateTime.hpp"
#include "LogFile.hpp"

#include <string>
#include <vector>

const constexpr char *SKYSIGHT_USER_CLIENT = "OpenSoar";

SkysightAPIQueue::~SkysightAPIQueue() {
	LogFormat("SkysightAPIQueue::~SkysightAPIQueue %d", timer.IsActive());
  timer.Cancel();
}

bool 
SkysightAPIQueue::IsLoggedIn() {
  // the key is valid for 1000 sec ( = 16:40min)
  auto now = DateTime::now();
  return key_expiry_time >= now;
}

void 
SkysightAPIQueue::DoClearingQueue() {
  for (auto &&i = request_queue.begin(); i < request_queue.end(); /* ++i */) {
    if ((*i)->GetStatus() != SkysightRequest::Status::Busy) {
      (*i)->Done();
      i = request_queue.erase(i);
    } else {
      i = i++;
    }
  }
  timer.Cancel();
  is_clearing = false;
}

void 
SkysightAPIQueue::AddRequest(std::unique_ptr<SkysightAsyncRequest> request,
			     bool append_end)
{
  if (!append_end) {
    // f.e. Login requests jump to the front of the queue
    if (request_queue.empty() ||
        request->GetType() != SkysightCallType::Login ||
        request_queue.begin()->get()->GetType() != SkysightCallType::Login)
      request_queue.insert(request_queue.begin(), std::move(request));
    else 
      request->Done();
  } else {
    request_queue.emplace_back(std::move(request));
  }
  if (!is_busy)
    Process();
}

void
SkysightAPIQueue::AddLoginRequest(
  std::unique_ptr<SkysightAsyncRequest> request)
{
  // add LoginRequest at the head of the queue
  request_queue.insert(request_queue.begin(), std::move(request));
  if (!is_busy)
    Process();
}

#ifdef SKYSIGHT_FORECAST 
void
SkysightAPIQueue::AddDecodeJob(std::unique_ptr<CDFDecoder> &&job) {
  decode_queue.emplace_back(std::move(job));
  if(!is_busy)
    Process();
}
#endif  // SKYSIGHT_FORECAST 

void 
SkysightAPIQueue::Process()
{
  is_busy = true;

  if(is_clearing) {
    DoClearingQueue();
  } else if (!request_queue.empty()) {
    auto job = request_queue.begin();
    switch ((*job)->GetStatus()) {
    case SkysightRequest::Status::Idle:
      // Provide the job with the very latest API key just prior to execution
      if ((*job)->GetType() == SkysightCallType::Login) {
        (*job)->SetCredentials(SKYSIGHT_USER_CLIENT, email, password);
        (*job)->Process();
      } else {
        if (!IsLoggedIn()) {
          // inject a login request at the front of the queue
          SkysightAPI::GenerateLoginRequest();

        } else {
          (*job)->SetCredentials(key.c_str());
          (*job)->Process();
        }
      }

      if (!timer.IsActive())
	      timer.Schedule(std::chrono::milliseconds(300));
      break;
    case SkysightRequest::Status::Complete:
    case SkysightRequest::Status::Error:
      (*job)->Done();
      request_queue.erase(job);
      break;
    case SkysightRequest::Status::Busy:
      break;
    }
  }

#ifdef SKYSIGHT_FORECAST 
  if (!empty(decode_queue)) {
    auto &&decode_job = decode_queue.begin();
    switch ((*decode_job)->GetStatus()) {
    case CDFDecoder::Status::Idle:
      (*decode_job)->DecodeAsync();
      if (!timer.IsActive())
	      timer.Schedule(std::chrono::milliseconds(300));
      break;
    case CDFDecoder::Status::Complete:
    case CDFDecoder::Status::Error:
      (*decode_job)->Done();
      try
      {
        decode_queue.erase(decode_job);
      }
      catch (std::exception &e)
      {
	      LogError(std::current_exception(),
             e.what());
      }
      break;
    case CDFDecoder::Status::Busy:
      break;
    }
  }
#endif  // SKYSIGHT_FORECAST 

#ifdef SKYSIGHT_FORECAST 
  if (empty(request_queue) && empty(decode_queue))
#else  // SKYSIGHT_FORECAST 
  if (empty(request_queue))
#endif  // SKYSIGHT_FORECAST 
    timer.Cancel();

  is_busy = false;
}

bool
SkysightAPIQueue::SetKey(const std::string _key,
			 const uint64_t _key_expiry_time)
{
  key = _key;
  key_expiry_time = _key_expiry_time;

  return IsLoggedIn();
//  if (!IsLoggedIn())
//    SkysightAPI::GenerateLoginRequest();
}

void
SkysightAPIQueue::SetCredentials(const std::string_view _email,
				 const std::string_view _pass)
{
  password = _pass;
  email = _email;
}

void
SkysightAPIQueue::Clear(const std::string msg)
{
  LogFormat("SkysightAPIQueue::Clear %s", msg.c_str());
  is_clearing = true;
}
