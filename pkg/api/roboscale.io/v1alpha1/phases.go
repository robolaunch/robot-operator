package v1alpha1

type RobotPhase string

const (
	RobotPhaseCreatingEnvironment      RobotPhase = "CreatingEnvironment"
	RobotPhaseCreatingDiscoveryServer  RobotPhase = "CreatingDiscoveryServer"
	RobotPhaseConfiguringEnvironment   RobotPhase = "ConfiguringEnvironment"
	RobotPhaseCreatingBridge           RobotPhase = "CreatingBridge"
	RobotPhaseCreatingDevelopmentSuite RobotPhase = "CreatingDevelopmentSuite"
	RobotPhaseConfiguringWorkspaces    RobotPhase = "ConfiguringWorkspaces"
	RobotPhaseEnvironmentReady         RobotPhase = "EnvironmentReady"
	RobotPhaseBuilding                 RobotPhase = "Building"
	RobotPhaseBuilt                    RobotPhase = "Built"
	RobotPhaseLaunching                RobotPhase = "Launching"
	RobotPhaseRunning                  RobotPhase = "Running"
	RobotPhaseDeletingBridge           RobotPhase = "DeletingBridge"
	RobotPhaseDeletingDiscoveryServer  RobotPhase = "DeletingDiscoveryServer"
	RobotPhaseDeletingLoaderJob        RobotPhase = "DeletingLoaderJob"
	RobotPhaseDeletingVolumes          RobotPhase = "DeletingVolumes"

	RobotPhaseFailed RobotPhase = "Failed"
)

type DiscoveryServerPhase string

const (
	DiscoveryServerPhaseCreatingService       DiscoveryServerPhase = "CreatingService"
	DiscoveryServerPhaseCreatingPod           DiscoveryServerPhase = "CreatingPod"
	DiscoveryServerPhaseCreatingServiceExport DiscoveryServerPhase = "CreatingServiceExport"
	DiscoveryServerPhaseCreatingConfigMap     DiscoveryServerPhase = "CreatingConfigMap"
	DiscoveryServerPhaseReady                 DiscoveryServerPhase = "Ready"
	DiscoveryServerPhaseDeletingConfigMap     DiscoveryServerPhase = "DeletingConfigMap"
	DiscoveryServerPhaseDeletingPod           DiscoveryServerPhase = "DeletingPod"
	DiscoveryServerPhaseDeletingService       DiscoveryServerPhase = "DeletingService"
)

type BridgePhase string

const (
	BridgePhaseCreatingService BridgePhase = "CreatingService"
	BridgePhaseCreatingPod     BridgePhase = "CreatingPod"
	BridgePhaseReady           BridgePhase = "Ready"
	BridgePhaseDeletingPod     BridgePhase = "DeletingPod"
	BridgePhaseDeletingService BridgePhase = "DeletingService"
)

type WorkspaceManagerPhase string

const (
	WorkspaceManagerPhaseConfiguringWorkspaces WorkspaceManagerPhase = "ConfiguringWorkspaces"
	WorkspaceManagerPhaseReady                 WorkspaceManagerPhase = "Ready"
	WorkspaceManagerPhaseFailed                WorkspaceManagerPhase = "Failed"
)

type BuildManagerPhase string

const (
	BuildManagerRobotNotFound            BuildManagerPhase = "RobotNotFound"
	BuildManagerWaitingForOtherResources BuildManagerPhase = "WaitingForOtherResources"
	BuildManagerCreatingConfigMap        BuildManagerPhase = "CreatingConfigMap"
	BuildManagerBuildingRobot            BuildManagerPhase = "BuildingRobot"
	BuildManagerReady                    BuildManagerPhase = "Ready"
	BuildManagerFailed                   BuildManagerPhase = "Failed"
	BuildManagerDeactivating             BuildManagerPhase = "Deactivating"
	BuildManagerInactive                 BuildManagerPhase = "Inactive"
)

type LaunchManagerPhase string

const (
	LaunchManagerPhaseRobotNotFound LaunchManagerPhase = "RobotNotFound"
	LaunchManagerPhaseCreatingPod   LaunchManagerPhase = "CreatingPod"
	LaunchManagerPhaseLaunching     LaunchManagerPhase = "Launching"
	LaunchManagerPhaseReady         LaunchManagerPhase = "Ready"
	LaunchManagerPhaseDeactivating  LaunchManagerPhase = "Deactivating"
	LaunchManagerPhaseInactive      LaunchManagerPhase = "Inactive"
)

type RobotDevSuitePhase string

const (
	RobotDevSuitePhaseRobotNotFound    RobotDevSuitePhase = "RobotNotFound"
	RobotDevSuitePhaseCreatingRobotVDI RobotDevSuitePhase = "CreatingRobotVDI"
	RobotDevSuitePhaseCreatingRobotIDE RobotDevSuitePhase = "CreatingRobotIDE"
	RobotDevSuitePhaseRunning          RobotDevSuitePhase = "Running"
	RobotDevSuitePhaseDeactivating     RobotDevSuitePhase = "Deactivating"
	RobotDevSuitePhaseInactive         RobotDevSuitePhase = "Inactive"
)

type RobotIDEPhase string

const (
	RobotIDEPhaseCreatingService RobotIDEPhase = "CreatingService"
	RobotIDEPhaseCreatingPod     RobotIDEPhase = "CreatingPod"
	RobotIDEPhaseCreatingIngress RobotIDEPhase = "CreatingIngress"
	RobotIDEPhaseRunning         RobotIDEPhase = "Running"
)

type RobotVDIPhase string

const (
	RobotVDIPhaseCreatingPVC        RobotVDIPhase = "CreatingPVC"
	RobotVDIPhaseCreatingTCPService RobotVDIPhase = "CreatingTCPService"
	RobotVDIPhaseCreatingUDPService RobotVDIPhase = "CreatingUDPService"
	RobotVDIPhaseCreatingPod        RobotVDIPhase = "CreatingPod"
	RobotVDIPhaseCreatingIngress    RobotVDIPhase = "CreatingIngress"
	RobotVDIPhaseRunning            RobotVDIPhase = "Running"
)

type MetricsCollectorPhase string

const (
	MetricsCollectorPhaseRobotNotFound MetricsCollectorPhase = "RobotNotFound"
	MetricsCollectorPhaseRunning       MetricsCollectorPhase = "Running"
)
