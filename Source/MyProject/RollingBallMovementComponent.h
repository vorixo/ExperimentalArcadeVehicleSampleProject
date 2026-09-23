#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PawnMovementComponent.h"
#include "Chaos/SimCallbackObject.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "RollingBallMovementComponent.generated.h"

class FRollingBallAsync;

// Keeps the Blueprint component and control functions used by Boulder.
UCLASS(meta = (BlueprintSpawnableComponent))
class MYPROJECT_API URollingBallMovementComponent : public UPawnMovementComponent
{
	GENERATED_BODY()

public:
	URollingBallMovementComponent();
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable)
	void SetThrottleInput(float InThrottle);

	UFUNCTION(BlueprintCallable)
	void SetSteeringInput(float InSteering);

	UFUNCTION(BlueprintCallable)
	void SetTravelDirectionInput(FRotator InTravelDirection);

	UFUNCTION(BlueprintCallable)
	void Jump();

protected:
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void OnUnregister() override;

private:
	void StartSimulation();
	void StopSimulation();
	FRollingBallAsync* RollingBallAsync = nullptr;

	UPROPERTY()
	TObjectPtr<UNetworkPhysicsComponent> NetworkPhysicsComponent = nullptr;

	float ThrottleInput_External = 0.f;
	float SteeringInput_External = 0.f;
	FRotator TravelDirection_External = FRotator::ZeroRotator;
	uint32 JumpRequest_External = 0;
};

USTRUCT()
struct FNetworkBallInputs : public FNetworkPhysicsPayload
{
	GENERATED_BODY()

	UPROPERTY()
	float ThrottleInput = 0.f;

	UPROPERTY()
	float SteeringInput = 0.f;

	UPROPERTY()
	FRotator TravelDirection = FRotator::ZeroRotator;

	// Persistent request, so a press isn't lost between game and physics ticks.
	UPROPERTY()
	uint32 JumpRequest = 0;

	virtual void InterpolateData(const FNetworkPhysicsPayload& MinData,
		const FNetworkPhysicsPayload& MaxData, float LerpAlpha) override;
	virtual void MergeData(const FNetworkPhysicsPayload& FromData) override;
	virtual void DecayData(float DecayAmount) override;
	virtual bool CompareData(const FNetworkPhysicsPayload& PredictedData) const override;
};

USTRUCT()
struct FNetworkBallStates : public FNetworkPhysicsPayload
{
	GENERATED_BODY()

	// Records which request has already produced an action.
	UPROPERTY()
	uint32 LastJumpRequest = 0;

	virtual void InterpolateData(const FNetworkPhysicsPayload& MinData,
		const FNetworkPhysicsPayload& MaxData, float LerpAlpha) override;
	virtual bool CompareData(const FNetworkPhysicsPayload& PredictedData) const override;
};

// UE 5.8 addition: the jump effect is a networked physics action.
USTRUCT()
struct FBallJumpAction : public FNetworkPhysicsActionPayload
{
	GENERATED_BODY()

	UPROPERTY()
	uint32 JumpRequest = 0;

	virtual const EActionAuthorStyle GetAuthorStyle() const override
	{
		return EActionAuthorStyle::Predicted;
	}

	virtual bool IsNearlyEqual(const FNetworkPhysicsActionPayload& Other) const override
	{
		return JumpRequest == static_cast<const FBallJumpAction&>(Other).JumpRequest;
	}
};

struct FAsyncInputRollingBall : public Chaos::FSimCallbackInput
{
	float ThrottleInput = 0.f;
	float SteeringInput = 0.f;
	FRotator TravelDirection = FRotator::ZeroRotator;
	uint32 JumpRequest = 0;

	void Reset()
	{
		ThrottleInput = 0.f;
		SteeringInput = 0.f;
		TravelDirection = FRotator::ZeroRotator;
		JumpRequest = 0;
	}
};

struct FAsyncOutputRollingBall : public Chaos::FSimCallbackOutput
{
	void Reset() {}
};

class FRollingBallAsync : public Chaos::TSimCallbackObject<FAsyncInputRollingBall,
	FAsyncOutputRollingBall, Chaos::ESimCallbackOptions::Presimulate
	| Chaos::ESimCallbackOptions::PhysicsObjectUnregister | Chaos::ESimCallbackOptions::Rewind>
	, public TNetworkPhysicsInputState_Internal<FNetworkBallInputs, FNetworkBallStates>
	, public INetworkPhysicsActionHandler_Internal
{
	friend URollingBallMovementComponent;

	virtual void OnPostInitialize_Internal() override;
	virtual void ProcessInputs_Internal(int32 PhysicsStep) override;
	virtual void OnPreSimulate_Internal() override;
	virtual void OnPhysicsObjectUnregistered_Internal(Chaos::FConstPhysicsObjectHandle Object) override;

	virtual void BuildInput_Internal(FNetworkBallInputs& Input) const override;
	virtual void ValidateInput_Internal(FNetworkBallInputs& Input) const override;
	virtual void ApplyInput_Internal(const FNetworkBallInputs& Input) override;
	virtual void BuildState_Internal(FNetworkBallStates& State) const override;
	virtual void ApplyState_Internal(const FNetworkBallStates& State) override;
	virtual void ApplyAction_Internal(const TInstancedStruct<FNetworkPhysicsActionPayload>& Action) override;

	Chaos::FConstPhysicsObjectHandle PhysicsObject = nullptr;
	FAsyncNetworkPhysicsComponent* NetworkPhysics_Internal = nullptr;
	float ThrottleInput_Internal = 0.f;
	float SteeringInput_Internal = 0.f;
	FRotator TravelDirection_Internal = FRotator::ZeroRotator;
	uint32 JumpRequest_Internal = 0;
	uint32 LastJumpRequest_Internal = 0;
};
