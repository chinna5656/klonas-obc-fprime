/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * ParachuteDeployer Component Header
 * ============================================================================
 */

#ifndef OBC_PARACHUTEDEPLOYER_HPP_
#define OBC_PARACHUTEDEPLOYER_HPP_

#include "obc/Components/ParachuteDeployer/ParachuteDeployerComponentAc.hpp"

namespace Obc {

  class ParachuteDeployer : public ParachuteDeployerComponentBase {

    public:

      //! Safety arm passcode required to arm or force deploy
      static constexpr U32 SAFETY_ARM_KEY = 0xDEADBEEF;

      //! Default arm window timeout in seconds
      static constexpr U32 DEFAULT_ARM_TIMEOUT_SEC = 60;

      //! Default burn duration in milliseconds
      static constexpr U32 DEFAULT_BURN_MS = 3000;

      //! Maximum permissible burn time to protect hardware traces
      static constexpr U32 MAX_PERMISSIBLE_BURN_MS = 5000;

      //! Deployment state machine enumeration
      enum State : U8 {
        STATE_DISARMED = 0,
        STATE_ARMED    = 1,
        STATE_BURNING  = 2,
        STATE_DEPLOYED = 3,
        STATE_FAULT    = 4
      };

      //! Construct ParachuteDeployer instance
      ParachuteDeployer(const char* const compName);

      //! Destructor
      ~ParachuteDeployer() override = default;

      //! Initialize component
      void init(FwEnumStoreType instance = 0);

    private:

      // ----------------------------------------------------------------------
      // Handlers for input ports
      // ----------------------------------------------------------------------

      //! Handler for deployment trigger port (void return)
      void deployIn_handler(
          FwIndexType portNum,
          U32 confidence,
          bool force
      ) override;

      //! Periodic tick handler for timing state transitions
      void schedIn_handler(
          FwIndexType portNum,
          U32 context
      ) override;

      // ----------------------------------------------------------------------
      // Command handlers
      // ----------------------------------------------------------------------

      void PARACHUTE_ARM_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          U32 key
      ) override;

      void PARACHUTE_DISARM_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq
      ) override;

      void PARACHUTE_DEPLOY_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          U32 key
      ) override;

      void PARACHUTE_SET_BURN_TIME_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          U32 burnDurationMs
      ) override;

      // ----------------------------------------------------------------------
      // Internal state machine routines
      // ----------------------------------------------------------------------

      //! Initiate thermal burn sequence
      void startBurn();

      //! Terminate thermal burn sequence
      void stopBurn();

      // ----------------------------------------------------------------------
      // Member variables (zero dynamic allocation)
      // ----------------------------------------------------------------------

      State m_state;
      U32 m_burnDurationMs;
      U32 m_burnRemainingMs;
      U32 m_armTimeoutSec;
      U32 m_deploymentCount;

  };

}

#endif /* OBC_PARACHUTEDEPLOYER_HPP_ */
