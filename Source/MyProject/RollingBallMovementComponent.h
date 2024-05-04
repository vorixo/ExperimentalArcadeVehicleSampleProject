// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/PawnMovementComponent.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "RollingBallMovementComponent.generated.h"


/** Ball inputs from the player controller */
USTRUCT()
struct FBallInputs
{
	GENERATED_BODY()

	FBallInputs()
		: SteeringInput(0.f)
		, ThrottleInput(0.f)
		, TravelDirection(FRotator::ZeroRotator)
		, JumpCount(0)
	{}

	// Steering output to physics system. Range -1...1
	UPROPERTY()
	float SteeringInput;

	// Accelerator output to physics system. Range -1...1
	UPROPERTY()
	float ThrottleInput;

	// Desired direction
	UPROPERTY()
	FRotator TravelDirection;

	/** Counter for user jumps */
	UPROPERTY()
	int32 JumpCount;
};

/** Ball state data that will be used in the state history to rewind the simulation at some point in time */
USTRUCT()
struct FNetworkBallStates : public FNetworkPhysicsData
{
	GENERATED_BODY()

	/**  Serialize data function that will be used to transfer the struct across the network */
	bool NetSerialize(FArchive& Ar, class UPackageMap* Map, bool& bOutSuccess);
};

template<>
struct TStructOpsTypeTraits<FNetworkBallStates> : public TStructOpsTypeTraitsBase2<FNetworkBallStates>
{
	enum
	{
		WithNetSerializer = true,
	};
};

/** Ball Inputs data that will be used in the inputs history to be applied while simulating */
USTRUCT()
struct FNetworkBallInputs : public FNetworkPhysicsData
{
	GENERATED_BODY()

	/** List of incoming control inputs coming from the local client */
	UPROPERTY()
	FBallInputs BallInputs;

	/**  Apply the data onto the network physics component */
	virtual void ApplyData(UActorComponent* NetworkComponent) const override;

	/**  Build the data from the network physics component */
	virtual void BuildData(const UActorComponent* NetworkComponent) override;

	/**  Serialize data function that will be used to transfer the struct across the network */
	bool NetSerialize(FArchive& Ar, class UPackageMap* Map, bool& bOutSuccess);

	/** Interpolate the data in between two inputs data */
	virtual void InterpolateData(const FNetworkPhysicsData& MinData, const FNetworkPhysicsData& MaxData) override;

	/** Merge data into this input */
	virtual void MergeData(const FNetworkPhysicsData& FromData) override;
};

template<>
struct TStructOpsTypeTraits<FNetworkBallInputs> : public TStructOpsTypeTraitsBase2<FNetworkBallInputs>
{
	enum
	{
		WithNetSerializer = true,
	};
};

struct FPhysicsBallTraits
{
	using InputsType = FNetworkBallInputs;
	using StatesType = FNetworkBallStates;
};


UCLASS(meta = (BlueprintSpawnableComponent))
class MYPROJECT_API URollingBallMovementComponent : public UPawnMovementComponent
{
	GENERATED_UCLASS_BODY()
	
public:

	/** Overridden to allow registration with components NOT owned by a Pawn. */
	virtual void SetUpdatedComponent(USceneComponent* NewUpdatedComponent) override;

	/** Return true if it's suitable to create a physics representation of the Ball at this time */
	virtual bool ShouldCreatePhysicsState() const override;

	/** Used to create any physics engine information for this component */
	virtual void OnCreatePhysicsState() override;

	/** Used to shut down and physics engine structure for this component */
	virtual void OnDestroyPhysicsState() override;

	void InitializeBall();

	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime);

	UFUNCTION(BlueprintCallable)
	void SetThrottleInput(float InThrottle);

	UFUNCTION(BlueprintCallable)
	void SetSteeringInput(float InSteering);

	UFUNCTION(BlueprintCallable)
	void SetTravelDirectionInput(FRotator InTravelDirection);

	UFUNCTION(BlueprintCallable)
	void Jump();

public:

	// Ball inputs
	FBallInputs BallInputs;

	// Ball state
	FTransform BallWorldTransform;
	FVector BallForwardAxis;
	FVector BallRightAxis;

private:

	UPROPERTY()
	TObjectPtr<UNetworkPhysicsComponent> NetworkPhysicsComponent = nullptr;

	Chaos::FRigidBodyHandle_Internal* RigidHandle;
	FBodyInstance* BodyInstance;
	int32 PreviousJumpCount = 0;
};
