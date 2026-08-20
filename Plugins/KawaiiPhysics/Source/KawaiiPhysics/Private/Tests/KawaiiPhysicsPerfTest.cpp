// Copyright 2019-2026 pafuhana1213. All Rights Reserved.

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "HAL/PlatformTime.h"
#include "Templates/Function.h"
#include "Curves/CurveFloat.h"
#include "ExternalForces/KawaiiPhysicsExternalForce.h"
#include "KawaiiPhysicsTestHarness.h"

namespace
{
	constexpr int32 GWarmupFrames = 100;
	constexpr int32 GMeasureFrames = 2000;
	constexpr int32 GTrials = 5;
	constexpr float GFrameDt = 1.0f / 90.0f;
	constexpr double GAverageSubsteps = 60.0 / 90.0; // 1/90秒を1/60秒固定ステップへ蓄積する理論平均。

	FKawaiiPhysicsSettings MakePerfSettings(const float Radius = 2.0f)
	{
		FKawaiiPhysicsSettings Settings;
		Settings.Damping = 0.15f;
		Settings.WorldDampingLocation = 0.2f;
		Settings.WorldDampingRotation = 0.3f;
		Settings.Stiffness = 0.07f;
		Settings.Radius = Radius;
		Settings.LimitAngle = 0.0f;
		return Settings;
	}

	void ConfigureBaseSimulation(FKawaiiPhysicsTestAccessor& A, const float Radius = 2.0f)
	{
		A.SetAllPhysicsSettings(MakePerfSettings(Radius));
		A.SetSimulationSpace(EKawaiiPhysicsSimulationSpace::ComponentSpace);
		A.SetGravityInSimSpace(FVector(0.0, 0.0, -980.0));
		A.SetFixedSubstepping(true, 60, 4);
		// 常に横移動させ、チェーンが静止解に貼り付いたまま計測されるのを避ける。
		A.SetSkelCompMove(FVector(0.3f, 0.0f, 0.0f), FQuat::Identity);
	}

	void AddPerfCollisionLimits(FKawaiiPhysicsTestAccessor& A)
	{
		for (int32 Index = 0; Index < 4; ++Index)
		{
			FSphericalLimit Sphere;
			Sphere.bEnable = true;
			Sphere.Location = FVector(2.0, 0.0, -100.0 - 180.0 * Index);
			Sphere.Rotation = FQuat::Identity;
			Sphere.Radius = 10.0f;
			Sphere.LimitType = ESphericalLimitType::Outer;
			A.Node.SphericalLimits.Add(Sphere);
		}

		for (int32 Index = 0; Index < 8; ++Index)
		{
			FCapsuleLimit Capsule;
			Capsule.bEnable = true;
			Capsule.Location = FVector(0.0, 2.0, -60.0 - 105.0 * Index);
			Capsule.Rotation = FQuat::Identity;
			Capsule.Radius = 5.0f;
			Capsule.Length = 80.0f;
			A.Node.CapsuleLimits.Add(Capsule);
		}

		for (int32 Index = 0; Index < 4; ++Index)
		{
			FBoxLimit Box;
			Box.bEnable = true;
			Box.Location = FVector(0.0, -2.0, -150.0 - 190.0 * Index);
			Box.Rotation = FQuat::Identity;
			Box.Extent = FVector(8.0, 8.0, 20.0);
			A.Node.BoxLimits.Add(Box);
		}

		for (int32 Index = 0; Index < 2; ++Index)
		{
			FPlanarLimit Planar;
			Planar.bEnable = true;
			Planar.Location = FVector(0.0, 0.0, -350.0 - 350.0 * Index);
			Planar.Rotation = FQuat::Identity;
			Planar.Plane = FPlane(Planar.Location, Planar.Rotation.GetUpVector());
			A.Node.PlanarLimits.Add(Planar);
		}
	}

	bool RunSimulationPerf(FAutomationTestBase& Test, const TCHAR* TestName,
	                       const TFunction<void(FKawaiiPhysicsTestAccessor&)>& Setup)
	{
		TArray<double> MsPerFrameValues;
		MsPerFrameValues.Reserve(GTrials);
		double Checksum = 0.0;
		int32 BoneCount = 0;
		bool bFinite = true;

		for (int32 Trial = 0; Trial < GTrials; ++Trial)
		{
			FKawaiiPhysicsTestAccessor A;
			Setup(A);
			BoneCount = A.Num();

			for (int32 Frame = 0; Frame < GWarmupFrames; ++Frame)
			{
				A.StepFrame(GFrameDt);
			}

			const double StartSeconds = FPlatformTime::Seconds();
			double TrialChecksum = 0.0;
			for (int32 Frame = 0; Frame < GMeasureFrames; ++Frame)
			{
				A.StepFrame(GFrameDt);
				const FVector Tip = A.TipLocation();
				TrialChecksum += Tip.X + Tip.Y + Tip.Z;
			}
			const double ElapsedSeconds = FPlatformTime::Seconds() - StartSeconds;
			const double MsPerFrame = ElapsedSeconds * 1000.0 / static_cast<double>(GMeasureFrames);

			Test.AddInfo(FString::Printf(TEXT("PERF_RAW %s trial=%d ms=%.6f"), TestName, Trial, MsPerFrame));
			MsPerFrameValues.Add(MsPerFrame);
			Checksum += TrialChecksum;

			if (!A.AllFinite())
			{
				Test.AddError(FString::Printf(TEXT("PERF %s produced NaN or Inf"), TestName));
				bFinite = false;
			}
		}

		MsPerFrameValues.Sort();
		const double MedianMsPerFrame = MsPerFrameValues[GTrials / 2];
		const double NsPerBoneStep = MedianMsPerFrame * 1000000.0 /
			FMath::Max(1.0, static_cast<double>(BoneCount) * GAverageSubsteps);
		Test.AddInfo(FString::Printf(
			TEXT("PERF %s median_ms_per_frame=%.6f ns_per_bone_step=%.3f checksum=%.6f"),
			TestName, MedianMsPerFrame, NsPerBoneStep, Checksum));
		return bFinite;
	}

	void FillLengthRate(FKawaiiPhysicsTestAccessor& A)
	{
		const int32 LastIndex = FMath::Max(1, A.Num() - 1);
		for (int32 Index = 0; Index < A.Num(); ++Index)
		{
			A.Bone(Index).LengthRateFromRoot = static_cast<float>(Index) / static_cast<float>(LastIndex);
		}
	}

	bool PhysicsSettingsFinite(const FKawaiiPhysicsTestAccessor& A)
	{
		for (int32 Index = 0; Index < A.Num(); ++Index)
		{
			const FKawaiiPhysicsSettings& Settings = A.Bone(Index).PhysicsSettings;
			if (!FMath::IsFinite(Settings.Damping) ||
				!FMath::IsFinite(Settings.WorldDampingLocation) ||
				!FMath::IsFinite(Settings.WorldDampingRotation) ||
				!FMath::IsFinite(Settings.Stiffness) ||
				!FMath::IsFinite(Settings.Radius) ||
				!FMath::IsFinite(Settings.LimitAngle))
			{
				return false;
			}
		}
		return true;
	}

	bool RunPhysicsSettingsPerf(FAutomationTestBase& Test, const TCHAR* TestName, const bool bSetDampingCurve)
	{
		constexpr int32 Calls = 20000;
		TArray<double> MsPerCallValues;
		MsPerCallValues.Reserve(GTrials);
		double Checksum = 0.0;
		bool bFinite = true;

		for (int32 Trial = 0; Trial < GTrials; ++Trial)
		{
			FKawaiiPhysicsTestAccessor A;
			A.BuildVerticalChain(200, 5.0f);
			FillLengthRate(A);
			A.Node.PhysicsSettings = MakePerfSettings(2.0f);
			if (bSetDampingCurve)
			{
				FRichCurve* Curve = A.Node.DampingCurveData.GetRichCurve();
				Curve->Reset();
				Curve->AddKey(0.0f, 0.5f);
				Curve->AddKey(1.0f, 1.5f);
			}

			const double StartSeconds = FPlatformTime::Seconds();
			double TrialChecksum = 0.0;
			for (int32 Call = 0; Call < Calls; ++Call)
			{
				A.CallUpdatePhysicsSettings();
				const FKawaiiPhysicsSettings& TipSettings = A.Bone(A.Num() - 1).PhysicsSettings;
				TrialChecksum += TipSettings.Damping + TipSettings.WorldDampingLocation +
					TipSettings.WorldDampingRotation + TipSettings.Stiffness + TipSettings.Radius +
					TipSettings.LimitAngle;
			}
			const double ElapsedSeconds = FPlatformTime::Seconds() - StartSeconds;
			const double MsPerCall = ElapsedSeconds * 1000.0 / static_cast<double>(Calls);

			Test.AddInfo(FString::Printf(TEXT("PERF_RAW %s trial=%d ms=%.6f"), TestName, Trial, MsPerCall));
			MsPerCallValues.Add(MsPerCall);
			Checksum += TrialChecksum;

			if (!PhysicsSettingsFinite(A))
			{
				Test.AddError(FString::Printf(TEXT("PERF %s produced NaN or Inf"), TestName));
				bFinite = false;
			}
		}

		MsPerCallValues.Sort();
		const double MedianMsPerCall = MsPerCallValues[GTrials / 2];
		const double NsPerBone = MedianMsPerCall * 1000000.0 / 200.0;
		Test.AddInfo(FString::Printf(
			TEXT("PERF %s median_ms_per_frame=%.6f ns_per_bone_step=%.3f checksum=%.6f"),
			TestName, MedianMsPerCall, NsPerBone, Checksum));
		return bFinite;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FKawaiiPhysicsPerfChainTest,
                                 "KawaiiPhysics.Perf.Chain",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FKawaiiPhysicsPerfChainTest::RunTest(const FString& Parameters)
{
	return RunSimulationPerf(*this, TEXT("KawaiiPhysics.Perf.Chain"),
		[](FKawaiiPhysicsTestAccessor& A)
		{
			A.BuildVerticalChain(200, 5.0f);
			ConfigureBaseSimulation(A);
		});
}

// legacy（サブステップOFF）。Exponent = TargetFramerate * DeltaTime となり 1.0f にならないため、
// 固定サブステップ時のように powf の y==1 特殊ケースへ落ちない。Stiffness の Pow コストはここで初めて現れる。
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FKawaiiPhysicsPerfChainLegacyTest,
                                 "KawaiiPhysics.Perf.ChainLegacy",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FKawaiiPhysicsPerfChainLegacyTest::RunTest(const FString& Parameters)
{
	return RunSimulationPerf(*this, TEXT("KawaiiPhysics.Perf.ChainLegacy"),
		[](FKawaiiPhysicsTestAccessor& A)
		{
			A.BuildVerticalChain(200, 5.0f);
			A.SetAllPhysicsSettings(MakePerfSettings(2.0f));
			A.SetSimulationSpace(EKawaiiPhysicsSimulationSpace::ComponentSpace);
			A.SetGravityInSimSpace(FVector(0.0, 0.0, -980.0));
			A.SetFixedSubstepping(false, 60, 4);
			A.SetSkelCompMove(FVector(0.3f, 0.0f, 0.0f), FQuat::Identity);
		});
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FKawaiiPhysicsPerfCollisionTest,
                                 "KawaiiPhysics.Perf.Collision",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FKawaiiPhysicsPerfCollisionTest::RunTest(const FString& Parameters)
{
	return RunSimulationPerf(*this, TEXT("KawaiiPhysics.Perf.Collision"),
		[](FKawaiiPhysicsTestAccessor& A)
		{
			A.BuildVerticalChain(200, 5.0f);
			ConfigureBaseSimulation(A, 3.0f);
			AddPerfCollisionLimits(A);
		});
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FKawaiiPhysicsPerfConstraintTest,
                                 "KawaiiPhysics.Perf.Constraint",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FKawaiiPhysicsPerfConstraintTest::RunTest(const FString& Parameters)
{
	return RunSimulationPerf(*this, TEXT("KawaiiPhysics.Perf.Constraint"),
		[](FKawaiiPhysicsTestAccessor& A)
		{
			A.BuildTwoVerticalChains(100, 5.0f, 8.0f);
			ConfigureBaseSimulation(A);
			A.SetBoneConstraintIterations(4, 4);
			A.SetBoneConstraintGlobalComplianceType(EXPBDComplianceType::Leather);
			// 制約長を実際の横間隔(8)より短くして違反量を常に非ゼロにする（早期returnで計測が痩せるのを防ぐ）。
			for (int32 Depth = 0; Depth < 100; ++Depth)
			{
				A.AddRuntimeBoneConstraint(Depth, 100 + Depth, 6.0f);
			}
		});
}

// 拘束計算そのものを支配的にした重量ベンチ。1000ボーン / 999拘束 / 反復16+16。
// 制約毎の除算やコンプライアンス表引きのような小さな差を、ボーン側の処理に埋もれさせずに測るためのもの。
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FKawaiiPhysicsPerfConstraintHeavyTest,
                                 "KawaiiPhysics.Perf.ConstraintHeavy",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FKawaiiPhysicsPerfConstraintHeavyTest::RunTest(const FString& Parameters)
{
	return RunSimulationPerf(*this, TEXT("KawaiiPhysics.Perf.ConstraintHeavy"),
		[](FKawaiiPhysicsTestAccessor& A)
		{
			constexpr int32 PerChain = 500;
			A.BuildTwoVerticalChains(PerChain, 5.0f, 8.0f);
			ConfigureBaseSimulation(A);
			A.SetBoneConstraintIterations(16, 16);
			A.SetBoneConstraintGlobalComplianceType(EXPBDComplianceType::Leather);
			// 横方向と斜め方向の両方を張り、ボーン数に対して拘束数を稼ぐ。長さは実距離より短くして常に違反させる。
			for (int32 Depth = 0; Depth < PerChain; ++Depth)
			{
				A.AddRuntimeBoneConstraint(Depth, PerChain + Depth, 6.0f);
			}
			for (int32 Depth = 0; Depth < PerChain - 1; ++Depth)
			{
				A.AddRuntimeBoneConstraint(Depth, PerChain + Depth + 1, 7.0f);
			}
		});
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FKawaiiPhysicsPerfPhysicsSettingsTest,
                                 "KawaiiPhysics.Perf.PhysicsSettings",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FKawaiiPhysicsPerfPhysicsSettingsTest::RunTest(const FString& Parameters)
{
	bool bOk = true;
	bOk &= RunPhysicsSettingsPerf(*this, TEXT("KawaiiPhysics.Perf.PhysicsSettings.CurvesEmpty"), false);
	bOk &= RunPhysicsSettingsPerf(*this, TEXT("KawaiiPhysics.Perf.PhysicsSettings.CurvesSet"), true);
	return bOk;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FKawaiiPhysicsPerfSizeofTest,
                                 "KawaiiPhysics.Perf.Sizeof",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FKawaiiPhysicsPerfSizeofTest::RunTest(const FString& Parameters)
{
	AddInfo(FString::Printf(TEXT("SIZEOF FKawaiiPhysicsModifyBone = %d"),
	                        static_cast<int32>(sizeof(FKawaiiPhysicsModifyBone))));
	AddInfo(FString::Printf(TEXT("SIZEOF FKawaiiPhysicsSettings = %d"),
	                        static_cast<int32>(sizeof(FKawaiiPhysicsSettings))));
	AddInfo(FString::Printf(TEXT("SIZEOF FSphericalLimit = %d"), static_cast<int32>(sizeof(FSphericalLimit))));
	AddInfo(FString::Printf(TEXT("SIZEOF FCapsuleLimit = %d"), static_cast<int32>(sizeof(FCapsuleLimit))));
	AddInfo(FString::Printf(TEXT("SIZEOF FBoxLimit = %d"), static_cast<int32>(sizeof(FBoxLimit))));
	AddInfo(FString::Printf(TEXT("SIZEOF FPlanarLimit = %d"), static_cast<int32>(sizeof(FPlanarLimit))));
	AddInfo(FString::Printf(TEXT("SIZEOF FModifyBoneConstraint = %d"),
	                        static_cast<int32>(sizeof(FModifyBoneConstraint))));
	AddInfo(FString::Printf(TEXT("SIZEOF FKawaiiPhysics_ExternalForce = %d"),
	                        static_cast<int32>(sizeof(FKawaiiPhysics_ExternalForce))));
	return true;
}

#endif
