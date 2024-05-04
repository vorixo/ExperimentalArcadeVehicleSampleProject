// Fill out your copyright notice in the Description page of Project Settings.

#include "RollingBallMovementComponent.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"


URollingBallMovementComponent::URollingBallMovementComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	SetIsReplicatedByDefault(true);

	static const FName NetworkPhysicsComponentName(TEXT("PC_NetworkPhysicsComponent"));
	NetworkPhysicsComponent = CreateDefaultSubobject<UNetworkPhysicsComponent, UNetworkPhysicsComponent>(NetworkPhysicsComponentName);
	NetworkPhysicsComponent->SetNetAddressable(); // Make DSO components net addressable
	NetworkPhysicsComponent->SetIsReplicated(true);
}

void URollingBallMovementComponent::SetUpdatedComponent(USceneComponent* NewUpdatedComponent)
{
	UNavMovementComponent::SetUpdatedComponent(NewUpdatedComponent);
	PawnOwner = NewUpdatedComponent ? Cast<APawn>(NewUpdatedComponent->GetOwner()) : nullptr;
}

bool URollingBallMovementComponent::ShouldCreatePhysicsState() const
{
	if (!IsRegistered() || IsBeingDestroyed())
	{
		return false;
	}

	// only create 'Physics' Ball in game
	UWorld* World = GetWorld();
	if (World->IsGameWorld())
	{
		FPhysScene* PhysScene = World->GetPhysicsScene();

		if (PhysScene && UpdatedComponent && UpdatedPrimitive)
		{
			return true;	
		}
	}

	return false;
}

void URollingBallMovementComponent::OnCreatePhysicsState()
{
	Super::OnCreatePhysicsState();

	// only create Physics Ball in game
	UWorld* World = GetWorld();
	if (World->IsGameWorld())
	{
		InitializeBall();
			
		if (NetworkPhysicsComponent)
		{
			NetworkPhysicsComponent->CreateDataHistory<FPhysicsBallTraits>(this);
		}
	}

	// Initializing phys handle
	if (UPrimitiveComponent* Mesh = Cast<UPrimitiveComponent>(UpdatedComponent))
	{
		BodyInstance = Mesh->GetBodyInstance();
	}	
}

void URollingBallMovementComponent::OnDestroyPhysicsState()
{
	Super::OnDestroyPhysicsState();

	if (UpdatedComponent)
	{
		UpdatedComponent->RecreatePhysicsState();
	}
	if (NetworkPhysicsComponent)
	{
		NetworkPhysicsComponent->RemoveDataHistory();
	}
}

void URollingBallMovementComponent::InitializeBall()
{
	if (UpdatedComponent && UpdatedPrimitive)
	{
		SetAsyncPhysicsTickEnabled(true);
	}
}

void URollingBallMovementComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);

	if (!BodyInstance)
	{
		return;
	}

	if (const auto Handle = BodyInstance->ActorHandle)
	{
		RigidHandle = Handle->GetPhysicsThreadAPI();
	}
	
	UWorld* World = GetWorld();
	if (World && RigidHandle)
	{
		const FTransform WorldTM(RigidHandle->R(), RigidHandle->X());
		BallWorldTransform = WorldTM;
		BallForwardAxis = BallWorldTransform.GetUnitAxis(EAxis::X);
		BallRightAxis = BallWorldTransform.GetUnitAxis(EAxis::Y);

		const FVector DesiredForwardVector = BallInputs.TravelDirection.Vector();
		const FVector DesiredRightVector = FRotationMatrix(BallInputs.TravelDirection).GetScaledAxis(EAxis::Y);

		// Update the simulation forces/impulses...
		if (BallInputs.JumpCount != PreviousJumpCount)
		{
			RigidHandle->SetLinearImpulse(FVector(0,0,500.f), true);
		}

		RigidHandle->AddForce(BallInputs.ThrottleInput * DesiredForwardVector * 80000.f, true);
		RigidHandle->AddForce(BallInputs.SteeringInput * DesiredRightVector * 80000.f, false);

		// Set prev frame vars
		PreviousJumpCount = BallInputs.JumpCount;
	}
}

void URollingBallMovementComponent::SetThrottleInput(float InThrottle)
{
	const float FinalThrottle = FMath::Clamp(InThrottle, -1.f, 1.f);
	BallInputs.ThrottleInput = InThrottle;
}

void URollingBallMovementComponent::SetSteeringInput(float InSteering)
{
	const float FinalSteering = FMath::Clamp(InSteering, -1.f, 1.f);
	BallInputs.SteeringInput = InSteering;
}

void URollingBallMovementComponent::SetTravelDirectionInput(FRotator InTravelDirection)
{
	BallInputs.TravelDirection = InTravelDirection;
}

void URollingBallMovementComponent::Jump()
{
	BallInputs.JumpCount++;
}

bool FNetworkBallInputs::NetSerialize(FArchive& Ar, class UPackageMap* Map, bool& bOutSuccess)
{
	FNetworkPhysicsData::SerializeFrames(Ar);

	Ar << BallInputs.SteeringInput;
	Ar << BallInputs.ThrottleInput;
	Ar << BallInputs.TravelDirection;
	Ar << BallInputs.JumpCount;

	bOutSuccess = true;
	return bOutSuccess;
}

void FNetworkBallInputs::ApplyData(UActorComponent* NetworkComponent) const
{
	if (URollingBallMovementComponent* BallMover = Cast<URollingBallMovementComponent>(NetworkComponent))
	{
		BallMover->BallInputs = BallInputs;
	}
}

void FNetworkBallInputs::BuildData(const UActorComponent* NetworkComponent)
{
	if (NetworkComponent)
	{
		if (const URollingBallMovementComponent* BallMover = Cast<const URollingBallMovementComponent>(NetworkComponent))
		{
			BallInputs = BallMover->BallInputs;
		}
	}
}

void FNetworkBallInputs::InterpolateData(const FNetworkPhysicsData& MinData, const FNetworkPhysicsData& MaxData)
{
	const FNetworkBallInputs& MinInput = static_cast<const FNetworkBallInputs&>(MinData);
	const FNetworkBallInputs& MaxInput = static_cast<const FNetworkBallInputs&>(MaxData);

	const float LerpFactor = MaxInput.LocalFrame == LocalFrame
		? 1.0f / (MaxInput.LocalFrame - MinInput.LocalFrame + 1) // Merge from min into max
		: (LocalFrame - MinInput.LocalFrame) / (MaxInput.LocalFrame - MinInput.LocalFrame); // Interpolate from min to max


	BallInputs.ThrottleInput = FMath::Lerp(MinInput.BallInputs.ThrottleInput, MaxInput.BallInputs.ThrottleInput, LerpFactor);
	BallInputs.SteeringInput = FMath::Lerp(MinInput.BallInputs.SteeringInput, MaxInput.BallInputs.SteeringInput, LerpFactor);
	BallInputs.TravelDirection = FMath::Lerp(MinInput.BallInputs.TravelDirection, MaxInput.BallInputs.TravelDirection, LerpFactor);
	BallInputs.JumpCount = LerpFactor < 0.5 ? MinInput.BallInputs.JumpCount : MaxInput.BallInputs.JumpCount;
}

void FNetworkBallInputs::MergeData(const FNetworkPhysicsData& FromData)
{
	// Perform merge through InterpolateData
	InterpolateData(FromData, *this);
}

bool FNetworkBallStates::NetSerialize(FArchive& Ar, UPackageMap* Map, bool& bOutSuccess)
{
	FNetworkPhysicsData::SerializeFrames(Ar);
	return true;
}