//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "humanoid_wbc/WbcBase.h"

namespace ocs2
{
  namespace humanoid
  {
    class WeightedWbc : public WbcBase
    {
    public:
      using WbcBase::WbcBase;

      vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                      size_t mode, scalar_t period, bool mpc_update = false) override;

      void loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real) override;

    protected:
      virtual Task formulateConstraints();
      virtual Task formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);
      Task formulateStanceBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);

    private:
      scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_, weightComPos_,weightStanceLeg_;
      scalar_t weightBaseAccelXY_, weightBaseAccelHeight_, weightBaseAccelAngular_;

      vector_t last_qpSol;
    };

  } // namespace humanoid
} // namespace ocs2
