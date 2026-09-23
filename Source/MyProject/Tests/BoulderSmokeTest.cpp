// Run explicitly with -ExecCmds="Boulder.SmokeTest" in a client/server test process.
// Uses the real Boulder Blueprint and movement component, with gravity disabled
// to measure two predicted jump actions independently of contacts and damping.
#if !UE_BUILD_SHIPPING

#include "../RollingBallMovementComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Containers/Ticker.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMisc.h"

namespace BoulderSmokeTest
{
struct FRun
{
	double StartedAt = FPlatformTime::Seconds();
	TWeakObjectPtr<APawn> Pawn;
	float Age = 0.f;
	int32 Requests = 0;
	int32 LastLoggedSecond = -1;
	bool bInitialized = false;
	double ExitAt = 0.0;
	uint8 ExitCode = 0;

	bool Tick(float DeltaTime)
	{
		if (ExitAt > 0.0)
		{
			if (FPlatformTime::Seconds() < ExitAt) return true;
			FPlatformMisc::RequestExitWithStatus(false, ExitCode);
			return false;
		}
		if (FPlatformTime::Seconds() - StartedAt > 60.0)
		{
			UE_LOG(LogTemp, Error, TEXT("BOULDER_SMOKE timeout waiting for a possessed boulder"));
			FPlatformMisc::RequestExitWithStatus(false, 1);
			return false;
		}

		if (!Pawn.IsValid())
		{
			for (const FWorldContext& Context : GEngine->GetWorldContexts())
			{
				UWorld* World = Context.World();
				if (!World || !World->IsGameWorld()) continue;
				for (TActorIterator<APawn> It(World); It; ++It)
				{
					if (It->GetController() && It->FindComponentByClass<URollingBallMovementComponent>())
					{
						Pawn = *It;
						break;
					}
				}
			}
			if (!Pawn.IsValid()) return true;
		}

		APawn* Ball = Pawn.Get();
		UPrimitiveComponent* Body = Cast<UPrimitiveComponent>(Ball->GetRootComponent());
		URollingBallMovementComponent* Movement = Ball->FindComponentByClass<URollingBallMovementComponent>();
		if (!Body || !Body->IsSimulatingPhysics() || !Movement)
		{
			UE_LOG(LogTemp, Error, TEXT("BOULDER_SMOKE invalid simulating root or movement component"));
			FPlatformMisc::RequestExitWithStatus(false, 1);
			return false;
		}

		if (!bInitialized)
		{
			Body->SetEnableGravity(false);
			Body->SetLinearDamping(0.f);
			Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
			if (APlayerController* Controller = Cast<APlayerController>(Ball->GetController()))
			{
				Ball->DisableInput(Controller);
			}
			bInitialized = true;
			UE_LOG(LogTemp, Display, TEXT("BOULDER_SMOKE initialized class=%s role=%d local=%d"),
				*Ball->GetClass()->GetPathName(), int32(Ball->GetLocalRole()), Ball->IsLocallyControlled());
		}

		Age += DeltaTime;
		if (Ball->IsLocallyControlled())
		{
			Movement->SetThrottleInput(Age < 6.f ? 0.2f : 0.f);
			Movement->SetSteeringInput(0.f);
			Movement->SetTravelDirectionInput(FRotator::ZeroRotator);
			if ((Requests == 0 && Age >= 3.f) || (Requests == 1 && Age >= 5.f))
			{
				Movement->Jump();
				++Requests;
				UE_LOG(LogTemp, Display, TEXT("BOULDER_SMOKE request=%d age=%.3f"), Requests, Age);
			}
		}

		const FVector Velocity = Body->GetPhysicsLinearVelocity();
		if (FMath::FloorToInt(Age) != LastLoggedSecond)
		{
			LastLoggedSecond = FMath::FloorToInt(Age);
			UE_LOG(LogTemp, Display, TEXT("BOULDER_SMOKE role=%d age=%.3f vx=%.3f vz=%.3f"),
				int32(Ball->GetLocalRole()), Age, Velocity.X, Velocity.Z);
		}
		if (Age >= 12.f)
		{
			const bool bPassed = FMath::IsNearlyEqual(Velocity.Z, 1000.0, 1.0) && Velocity.X > 1.0;
			UE_LOG(LogTemp, Display, TEXT("BOULDER_SMOKE %s role=%d vx=%.3f vz=%.3f"),
				bPassed ? TEXT("PASS") : TEXT("FAIL"), int32(Ball->GetLocalRole()), Velocity.X, Velocity.Z);
			// Give the client time to finish before the dedicated server closes its connection.
			ExitCode = bPassed ? 0 : 1;
			ExitAt = FPlatformTime::Seconds() + (Ball->GetNetMode() == NM_DedicatedServer ? 2.0 : 0.0);
		}
		return true;
	}
};

static FAutoConsoleCommand Command(
	TEXT("Boulder.SmokeTest"),
	TEXT("Run the isolated boulder movement/jump smoke test, then exit this process."),
	FConsoleCommandDelegate::CreateLambda([]
	{
		TSharedRef<FRun> Run = MakeShared<FRun>();
		FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
			[Run](float DeltaTime) { return Run->Tick(DeltaTime); }));
	}));
}
#endif
