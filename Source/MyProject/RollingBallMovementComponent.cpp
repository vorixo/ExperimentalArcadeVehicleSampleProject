#include "RollingBallMovementComponent.h"

#include "Chaos/PhysicsObjectInternalInterface.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Pawn.h"
#include "PBDRigidsSolver.h"
#include "Physics/Experimental/PhysScene_Chaos.h"

URollingBallMovementComponent::URollingBallMovementComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	SetIsReplicatedByDefault(true);

	if (UPhysicsSettings::Get()->PhysicsPrediction.bEnablePhysicsPrediction)
	{
		NetworkPhysicsComponent = CreateDefaultSubobject<UNetworkPhysicsComponent>(TEXT("PC_NetworkPhysicsComponent"));
		// Register only after our input/state/action interfaces are ready.
		NetworkPhysicsComponent->bAutoRegister = false;
		NetworkPhysicsComponent->SetNetAddressable();
		NetworkPhysicsComponent->SetIsReplicated(true);
	}
}

void URollingBallMovementComponent::BeginPlay()
{
	Super::BeginPlay();
	StartSimulation();
}

void URollingBallMovementComponent::StartSimulation()
{
	if (RollingBallAsync || !GetWorld()->IsGameWorld())
	{
		return;
	}

	if (!UpdatedComponent)
	{
		SetUpdatedComponent(GetOwner()->GetRootComponent());
	}
	UPrimitiveComponent* RootSimulatedComponent = UpdatedPrimitive;
	FPhysScene* PhysScene = GetWorld()->GetPhysicsScene();
	if (!RootSimulatedComponent || !PhysScene || !PhysScene->GetSolver() || !NetworkPhysicsComponent)
	{
		return;
	}
	const Chaos::FConstPhysicsObjectHandle PhysicsObject = RootSimulatedComponent->GetPhysicsObjectByName(NAME_None);
	if (!PhysicsObject)
	{
		// A replicated Blueprint's root may finish creating its body after BeginPlay.
		// TickComponent retries without registering a callback with an empty handle.
		return;
	}

	// Create the handlers before registering the Network Physics component. Its
	// initialization marshals the input/state interface to the physics thread.
	RollingBallAsync = PhysScene->GetSolver()->CreateAndRegisterSimCallbackObject_External<FRollingBallAsync>();
	RollingBallAsync->PhysicsObject = PhysicsObject;
	NetworkPhysicsComponent->SetPhysicsObject(PhysicsObject);
	NetworkPhysicsComponent->CreateDataHistory<FNetworkBallInputs, FNetworkBallStates>(RollingBallAsync);
	NetworkPhysicsComponent->SetActionHandler(RollingBallAsync);
	NetworkPhysicsComponent->SetCompareInputToTriggerRewind(true);
	NetworkPhysicsComponent->SetCompareStateToTriggerRewind(true);
	NetworkPhysicsComponent->RegisterComponent();
	if (!NetworkPhysicsComponent->HasBeenInitialized())
	{
		NetworkPhysicsComponent->InitializeComponent();
	}
	// Set before the newly registered callback is handed to the physics thread.
	RollingBallAsync->NetworkPhysics_Internal = NetworkPhysicsComponent->GetNetworkPhysicsComponent_Internal();

}

void URollingBallMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!RollingBallAsync)
	{
		StartSimulation();
	}
	// Check local control BEFORE requesting a producer buffer, as in MBobbo's tutorial.
	if (RollingBallAsync && GetPawnOwner() && GetPawnOwner()->IsLocallyControlled())
	{
		if (FAsyncInputRollingBall* AsyncInput = RollingBallAsync->GetProducerInputData_External())
		{
			AsyncInput->ThrottleInput = ThrottleInput_External;
			AsyncInput->SteeringInput = SteeringInput_External;
			AsyncInput->TravelDirection = TravelDirection_External;
			AsyncInput->JumpRequest = JumpRequest_External;
		}
	}
}

void URollingBallMovementComponent::SetThrottleInput(float InThrottle)
{
	ThrottleInput_External = FMath::Clamp(InThrottle, -1.f, 1.f);
}

void URollingBallMovementComponent::SetSteeringInput(float InSteering)
{
	SteeringInput_External = FMath::Clamp(InSteering, -1.f, 1.f);
}

void URollingBallMovementComponent::SetTravelDirectionInput(FRotator InTravelDirection)
{
	TravelDirection_External = FRotator(0.0, FRotator::NormalizeAxis(InTravelDirection.Yaw), 0.0);
}

void URollingBallMovementComponent::Jump()
{
	++JumpRequest_External;
}

void URollingBallMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopSimulation();
	Super::EndPlay(EndPlayReason);
}

void URollingBallMovementComponent::OnUnregister()
{
	StopSimulation();
	Super::OnUnregister();
}

void URollingBallMovementComponent::StopSimulation()
{
	if (RollingBallAsync)
	{
		// Stop the network callback before releasing its input/state/action handler.
		if (NetworkPhysicsComponent)
		{
			NetworkPhysicsComponent->SetActionHandler(nullptr);
			NetworkPhysicsComponent->RemoveDataHistory();
			NetworkPhysicsComponent->UnregisterComponent();
			if (NetworkPhysicsComponent->HasBeenInitialized())
			{
				NetworkPhysicsComponent->UninitializeComponent();
			}
		}
		if (FPhysScene* PhysScene = GetWorld()->GetPhysicsScene())
		{
			if (Chaos::FPhysicsSolver* Solver = PhysScene->GetSolver())
			{
				Solver->UnregisterAndFreeSimCallbackObject_External(RollingBallAsync);
			}
		}
		RollingBallAsync = nullptr;
	}
}

void FNetworkBallInputs::InterpolateData(const FNetworkPhysicsPayload& MinData,
	const FNetworkPhysicsPayload& MaxData, float LerpAlpha)
{
	const FNetworkBallInputs& Min = static_cast<const FNetworkBallInputs&>(MinData);
	const FNetworkBallInputs& Max = static_cast<const FNetworkBallInputs&>(MaxData);
	ThrottleInput = FMath::Lerp(Min.ThrottleInput, Max.ThrottleInput, LerpAlpha);
	SteeringInput = FMath::Lerp(Min.SteeringInput, Max.SteeringInput, LerpAlpha);
	TravelDirection = FMath::Lerp(Min.TravelDirection, Max.TravelDirection, LerpAlpha);
	JumpRequest = LerpAlpha < 1.f ? Min.JumpRequest : Max.JumpRequest;
}

void FNetworkBallInputs::MergeData(const FNetworkPhysicsPayload& FromData)
{
	const FNetworkBallInputs& From = static_cast<const FNetworkBallInputs&>(FromData);
	ThrottleInput = FMath::Max(ThrottleInput, From.ThrottleInput);
	SteeringInput = FMath::Max(SteeringInput, From.SteeringInput);
	// Retain the newer frame's direction and discrete request identifier.
}

void FNetworkBallInputs::DecayData(float DecayAmount)
{
	ThrottleInput *= 1.f - DecayAmount;
	SteeringInput *= 1.f - DecayAmount;
}

bool FNetworkBallInputs::CompareData(const FNetworkPhysicsPayload& PredictedData) const
{
	const FNetworkBallInputs& Predicted = static_cast<const FNetworkBallInputs&>(PredictedData);
	return ThrottleInput == Predicted.ThrottleInput
		&& SteeringInput == Predicted.SteeringInput
		&& TravelDirection == Predicted.TravelDirection
		&& JumpRequest == Predicted.JumpRequest;
}

void FNetworkBallStates::InterpolateData(const FNetworkPhysicsPayload& MinData,
	const FNetworkPhysicsPayload& MaxData, float LerpAlpha)
{
	const FNetworkBallStates& Min = static_cast<const FNetworkBallStates&>(MinData);
	const FNetworkBallStates& Max = static_cast<const FNetworkBallStates&>(MaxData);
	LastJumpRequest = LerpAlpha < 1.f ? Min.LastJumpRequest : Max.LastJumpRequest;
}

bool FNetworkBallStates::CompareData(const FNetworkPhysicsPayload& PredictedData) const
{
	return LastJumpRequest == static_cast<const FNetworkBallStates&>(PredictedData).LastJumpRequest;
}

void FRollingBallAsync::OnPostInitialize_Internal()
{
	if (PhysicsObject)
	{
		Chaos::FWritePhysicsObjectInterface_Internal Interface = Chaos::FPhysicsObjectInternalInterface::GetWrite();
		if (Chaos::FPBDRigidParticleHandle* ParticleHandle = Interface.GetRigidParticle(PhysicsObject))
		{
			ParticleHandle->SetSleepType(Chaos::ESleepType::NeverSleep);
		}
	}
}

void FRollingBallAsync::ProcessInputs_Internal(int32 PhysicsStep)
{
	const FAsyncInputRollingBall* AsyncInput = GetConsumerInput_Internal();
	if (!AsyncInput)
	{
		return;
	}
	Chaos::FPhysicsSolverBase* BaseSolver = GetSolver();
	if (!BaseSolver || BaseSolver->IsResimming())
	{
		// Historical/networked controls are supplied by ApplyInput_Internal.
		return;
	}
	ThrottleInput_Internal = AsyncInput->ThrottleInput;
	SteeringInput_Internal = AsyncInput->SteeringInput;
	TravelDirection_Internal = AsyncInput->TravelDirection;
	JumpRequest_Internal = AsyncInput->JumpRequest;
}

void FRollingBallAsync::OnPreSimulate_Internal()
{
	if (!PhysicsObject)
	{
		return;
	}
	Chaos::FWritePhysicsObjectInterface_Internal Interface = Chaos::FPhysicsObjectInternalInterface::GetWrite();
	Chaos::FPBDRigidParticleHandle* ParticleHandle = Interface.GetRigidParticle(PhysicsObject);
	if (!ParticleHandle)
	{
		return;
	}

	const FVector Forward = TravelDirection_Internal.Vector();
	const FVector Right = FRotationMatrix(TravelDirection_Internal).GetUnitAxis(EAxis::Y);
	const FVector Force = (ThrottleInput_Internal * Forward + SteeringInput_Internal * Right) * 80000.f;
	if (Force.SizeSquared() > UE_SMALL_NUMBER)
	{
		if (ParticleHandle->ObjectState() == Chaos::EObjectStateType::Sleeping)
		{
			static_cast<Chaos::FPhysicsSolver*>(GetSolver())->GetEvolution()->WakeParticle(ParticleHandle);
		}
		ParticleHandle->AddForce(Force, true);
	}

	if (NetworkPhysics_Internal && JumpRequest_Internal != LastJumpRequest_Internal)
	{
		FBallJumpAction Action;
		Action.JumpRequest = JumpRequest_Internal;
		// Author on both client and server, including during resimulation.
		const int32 NextFrame = static_cast<Chaos::FPhysicsSolver*>(GetSolver())->GetCurrentFrame() + 1;
		NetworkPhysics_Internal->EnqueueScheduledActionAtFrame_Internal(Action, uint32(0), NextFrame, true);
		LastJumpRequest_Internal = JumpRequest_Internal;
	}
}

void FRollingBallAsync::ApplyAction_Internal(const TInstancedStruct<FNetworkPhysicsActionPayload>& Action)
{
	if (!PhysicsObject || !Action.GetPtr<FBallJumpAction>())
	{
		return;
	}
	Chaos::FWritePhysicsObjectInterface_Internal Interface = Chaos::FPhysicsObjectInternalInterface::GetWrite();
	if (Chaos::FPBDRigidParticleHandle* ParticleHandle = Interface.GetRigidParticle(PhysicsObject))
	{
		// NeverSleep prevents automatic sleeping, but an already sleeping body must
		// still be woken through the evolution before a direct impulse is integrated.
		if (ParticleHandle->ObjectState() == Chaos::EObjectStateType::Sleeping)
		{
			static_cast<Chaos::FPhysicsSolver*>(GetSolver())->GetEvolution()->WakeParticle(ParticleHandle);
		}
		// The action, rather than the request, applies a 500 cm/s upward velocity change.
		ParticleHandle->SetLinearImpulseVelocity(
			ParticleHandle->LinearImpulseVelocity() + FVector(0, 0, 500), true);
	}
}

void FRollingBallAsync::OnPhysicsObjectUnregistered_Internal(Chaos::FConstPhysicsObjectHandle Object)
{
	if (PhysicsObject == Object)
	{
		PhysicsObject = nullptr;
	}
}

void FRollingBallAsync::BuildInput_Internal(FNetworkBallInputs& Input) const
{
	Input.ThrottleInput = ThrottleInput_Internal;
	Input.SteeringInput = SteeringInput_Internal;
	Input.TravelDirection = TravelDirection_Internal;
	Input.JumpRequest = JumpRequest_Internal;
}

void FRollingBallAsync::ValidateInput_Internal(FNetworkBallInputs& Input) const
{
	Input.ThrottleInput = FMath::Clamp(Input.ThrottleInput, -1.f, 1.f);
	Input.SteeringInput = FMath::Clamp(Input.SteeringInput, -1.f, 1.f);
	Input.TravelDirection = FRotator(0.0, FRotator::NormalizeAxis(Input.TravelDirection.Yaw), 0.0);
}

void FRollingBallAsync::ApplyInput_Internal(const FNetworkBallInputs& Input)
{
	ThrottleInput_Internal = Input.ThrottleInput;
	SteeringInput_Internal = Input.SteeringInput;
	TravelDirection_Internal = Input.TravelDirection;
	JumpRequest_Internal = Input.JumpRequest;
}

void FRollingBallAsync::BuildState_Internal(FNetworkBallStates& State) const
{
	State.LastJumpRequest = LastJumpRequest_Internal;
}

void FRollingBallAsync::ApplyState_Internal(const FNetworkBallStates& State)
{
	LastJumpRequest_Internal = State.LastJumpRequest;
}
