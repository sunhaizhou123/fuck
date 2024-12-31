#pragma once

#include <memory>
#include "cmder\CmderOrchestrator.hpp"
#include ".\core\PoseHandler.hpp"

namespace adas
{
    /*
        Executor的具体实现
    */
    class ExecutorImpl final : public Executor
    {
    public:
        explicit ExecutorImpl(const Pose &pose, CmderOrchestrator *orchestrator) noexcept;
        ~ExecutorImpl() noexcept = default;
        ExecutorImpl(const ExecutorImpl &) = delete;
        ExecutorImpl &operator=(const ExecutorImpl &) = delete;

    public:
        Pose Query(void) const noexcept override;
        void Execute(const std::string &commands) noexcept override;

    private:
        PoseHandler poseHandler;
        std::unique_ptr<CmderOrchestrator> orchestrator;
    };
}