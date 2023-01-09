package v1alpha1

import (
	"github.com/robolaunch/robot-operator/internal"
	"k8s.io/apimachinery/pkg/types"
)

// ********************************
// WorkspaceManager helpers
// ********************************

func (workspacemanager *WorkspaceManager) GetClonerJobMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      workspacemanager.Name + internal.JOB_CLONER_POSTFIX,
		Namespace: workspacemanager.Namespace,
	}
}

func (workspacemanager *WorkspaceManager) GetCleanupJobMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      workspacemanager.Name + internal.JOB_CLEANUP_POSTFIX,
		Namespace: workspacemanager.Namespace,
	}
}

// ********************************
// BuildManager helpers
// ********************************

func (buildmanager *BuildManager) GetConfigMapMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      buildmanager.Name + internal.CONFIG_MAP_SCRIPTS,
		Namespace: buildmanager.Namespace,
	}
}

// ********************************
// LaunchManager helpers
// ********************************

func (launchmanager *LaunchManager) GetLaunchPodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      launchmanager.Name + internal.POD_LAUNCH_POSTFIX,
		Namespace: launchmanager.Namespace,
	}
}
