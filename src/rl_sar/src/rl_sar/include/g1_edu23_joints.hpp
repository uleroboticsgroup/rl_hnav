#pragma once
#include <cstdint>

/**
 * Policy joint index (0–28) → EDU23 hardware availability
 *
 * Policy order (29 DoF):
 *
 *  0  LeftHipPitch          ✓
 *  1  LeftHipRoll           ✓
 *  2  LeftHipYaw            ✓
 *  3  LeftKnee              ✓
 *  4  LeftAnklePitch        ✓
 *  5  LeftAnkleRoll         ✓
 *
 *  6  RightHipPitch         ✓
 *  7  RightHipRoll          ✓
 *  8  RightHipYaw           ✓
 *  9  RightKnee             ✓
 * 10  RightAnklePitch       ✓
 * 11  RightAnkleRoll        ✓
 *
 * 12  WaistYaw              ✓
 * 13  WaistRoll             ✗ (locked on EDU23)
 * 14  WaistPitch            ✗ (locked on EDU23)
 *
 * 15  LeftShoulderPitch     ✓
 * 16  LeftShoulderRoll      ✓
 * 17  LeftShoulderYaw       ✓
 * 18  LeftElbow              ✓
 * 19  LeftWristRoll         ✓
 * 20  LeftWristPitch        ✗ (not present on EDU23)
 * 21  LeftWristYaw          ✗ (not present on EDU23)
 *
 * 22  RightShoulderPitch    ✓
 * 23  RightShoulderRoll     ✓
 * 24  RightShoulderYaw      ✓
 * 25  RightElbow             ✓
 * 26  RightWristRoll        ✓
 * 27  RightWristPitch       ✗ (not present on EDU23)
 * 28  RightWristYaw         ✗ (not present on EDU23)
 */

static constexpr bool is_edu23_active_joint(int policy_joint_index)
{
    switch (policy_joint_index)
    {
        // ===== LOWER BODY =====
        case 0:  // LeftHipPitch
        case 1:  // LeftHipRoll
        case 2:  // LeftHipYaw
        case 3:  // LeftKnee
        case 4:  // LeftAnklePitch
        case 5:  // LeftAnkleRoll

        case 6:  // RightHipPitch
        case 7:  // RightHipRoll
        case 8:  // RightHipYaw
        case 9:  // RightKnee
        case 10: // RightAnklePitch
        case 11: // RightAnkleRoll

        // ===== WAIST =====
        case 12: // WaistYaw

        // ===== LEFT ARM =====
        case 15: // LeftShoulderPitch
        case 16: // LeftShoulderRoll
        case 17: // LeftShoulderYaw
        case 18: // LeftElbow
        case 19: // LeftWristRoll

        // ===== RIGHT ARM =====
        case 22: // RightShoulderPitch
        case 23: // RightShoulderRoll
        case 24: // RightShoulderYaw
        case 25: // RightElbow
        case 26: // RightWristRoll
            return true;

        // ===== DISABLED ON EDU23 =====
        case 13: // WaistRoll (locked)
        case 14: // WaistPitch (locked)

        case 20: // LeftWristPitch (absent)
        case 21: // LeftWristYaw   (absent)

        case 27: // RightWristPitch (absent)
        case 28: // RightWristYaw   (absent)
            return false;

        default:
            return false;
    }
}
