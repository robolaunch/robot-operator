/*
Copyright 2022.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

package main

import (
	"flag"
	"os"

	// Import all Kubernetes client auth plugins (e.g. Azure, GCP, OIDC, etc.)
	// to ensure that exec-entrypoint and run can make use of them.
	"k8s.io/client-go/dynamic"
	_ "k8s.io/client-go/plugin/pkg/client/auth"

	"k8s.io/apimachinery/pkg/runtime"
	utilruntime "k8s.io/apimachinery/pkg/util/runtime"
	clientgoscheme "k8s.io/client-go/kubernetes/scheme"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/healthz"
	"sigs.k8s.io/controller-runtime/pkg/log/zap"

	mcsv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/external/apis/mcsv1alpha1/v1alpha1"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	buildManager "github.com/robolaunch/robot-operator/pkg/controllers/build_manager"
	launchManager "github.com/robolaunch/robot-operator/pkg/controllers/launch_manager"
	metrics "github.com/robolaunch/robot-operator/pkg/controllers/metrics"
	robot "github.com/robolaunch/robot-operator/pkg/controllers/robot"
	discoveryServer "github.com/robolaunch/robot-operator/pkg/controllers/robot/discovery_server"
	relayServer "github.com/robolaunch/robot-operator/pkg/controllers/robot/relay_server"
	rosBridge "github.com/robolaunch/robot-operator/pkg/controllers/robot/ros_bridge"
	robotDevSuite "github.com/robolaunch/robot-operator/pkg/controllers/robot_dev_suite"
	robotIDE "github.com/robolaunch/robot-operator/pkg/controllers/robot_dev_suite/robot_ide"
	robotVDI "github.com/robolaunch/robot-operator/pkg/controllers/robot_dev_suite/robot_vdi"
	workspaceManager "github.com/robolaunch/robot-operator/pkg/controllers/workspace_manager"
	//+kubebuilder:scaffold:imports
)

var (
	scheme   = runtime.NewScheme()
	setupLog = ctrl.Log.WithName("setup")
)

func init() {
	utilruntime.Must(clientgoscheme.AddToScheme(scheme))

	utilruntime.Must(robotv1alpha1.AddToScheme(scheme))

	utilruntime.Must(mcsv1alpha1.AddToScheme(scheme))
	//+kubebuilder:scaffold:scheme
}

func main() {
	var metricsAddr string
	var enableLeaderElection bool
	var probeAddr string
	flag.StringVar(&metricsAddr, "metrics-bind-address", ":8080", "The address the metric endpoint binds to.")
	flag.StringVar(&probeAddr, "health-probe-bind-address", ":8081", "The address the probe endpoint binds to.")
	flag.BoolVar(&enableLeaderElection, "leader-elect", false,
		"Enable leader election for controller manager. "+
			"Enabling this will ensure there is only one active controller manager.")
	opts := zap.Options{
		Development: true,
	}
	opts.BindFlags(flag.CommandLine)
	flag.Parse()

	ctrl.SetLogger(zap.New(zap.UseFlagOptions(&opts)))

	mgr, err := ctrl.NewManager(ctrl.GetConfigOrDie(), ctrl.Options{
		Scheme:                 scheme,
		MetricsBindAddress:     metricsAddr,
		Port:                   9443,
		HealthProbeBindAddress: probeAddr,
		LeaderElection:         enableLeaderElection,
		LeaderElectionID:       "ffc51351.roboscale.io",
		// LeaderElectionReleaseOnCancel defines if the leader should step down voluntarily
		// when the Manager ends. This requires the binary to immediately end when the
		// Manager is stopped, otherwise, this setting is unsafe. Setting this significantly
		// speeds up voluntary leader transitions as the new leader don't have to wait
		// LeaseDuration time first.
		//
		// In the default scaffold provided, the program ends immediately after
		// the manager stops, so would be fine to enable this option. However,
		// if you are doing or is intended to do any operation such as perform cleanups
		// after the manager stops then its usage might be unsafe.
		// LeaderElectionReleaseOnCancel: true,
	})
	if err != nil {
		setupLog.Error(err, "unable to start manager")
		os.Exit(1)
	}

	dynamicClient, err := dynamic.NewForConfig(mgr.GetConfig())
	if err != nil {
		setupLog.Error(err, "unable to create dynamic client")
	}

	// gvkPod := schema.GroupVersionKind{
	// 	Group:   "",
	// 	Version: "v1",
	// 	Kind:    "Pod",
	// }

	// restClient, err := apiutil.RESTClientForGVK(gvkPod, false, mgr.GetConfig(), serializer.NewCodecFactory(mgr.GetScheme()))
	// if err != nil {
	// 	setupLog.Error(err, "unable to create REST client")
	// }

	if err = (&robot.RobotReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "Robot")
		os.Exit(1)
	}
	if err = (&robotv1alpha1.Robot{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "Robot")
		os.Exit(1)
	}
	if err = (&discoveryServer.DiscoveryServerReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "DiscoveryServer")
		os.Exit(1)
	}
	if err = (&rosBridge.ROSBridgeReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "ROSBridge")
		os.Exit(1)
	}
	if err = (&buildManager.BuildManagerReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "BuildManager")
		os.Exit(1)
	}
	if err = (&robotv1alpha1.BuildManager{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "BuildManager")
		os.Exit(1)
	}
	if err = (&launchManager.LaunchManagerReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "LaunchManager")
		os.Exit(1)
	}
	if err = (&robotv1alpha1.LaunchManager{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "LaunchManager")
		os.Exit(1)
	}
	if err = (&robotDevSuite.RobotDevSuiteReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "RobotDevSuite")
		os.Exit(1)
	}
	if err = (&robotVDI.RobotVDIReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "RobotVDI")
		os.Exit(1)
	}
	if err = (&robotv1alpha1.RobotVDI{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "RobotVDI")
		os.Exit(1)
	}
	if err = (&robotIDE.RobotIDEReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "RobotIDE")
		os.Exit(1)
	}
	if err = (&robotv1alpha1.RobotIDE{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "RobotIDE")
		os.Exit(1)
	}
	if err = (&workspaceManager.WorkspaceManagerReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "WorkspaceManager")
		os.Exit(1)
	}
	if err = (&robotv1alpha1.WorkspaceManager{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "WorkspaceManager")
		os.Exit(1)
	}
	if err = (&robotv1alpha1.DiscoveryServer{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "DiscoveryServer")
		os.Exit(1)
	}
	if err = (&metrics.MetricsExporterReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "MetricsExporter")
		os.Exit(1)
	}
	if err = (&relayServer.RelayServerReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "RelayServer")
		os.Exit(1)
	}
	//+kubebuilder:scaffold:builder

	if err := mgr.AddHealthzCheck("healthz", healthz.Ping); err != nil {
		setupLog.Error(err, "unable to set up health check")
		os.Exit(1)
	}
	if err := mgr.AddReadyzCheck("readyz", healthz.Ping); err != nil {
		setupLog.Error(err, "unable to set up ready check")
		os.Exit(1)
	}

	setupLog.Info("starting manager")
	if err := mgr.Start(ctrl.SetupSignalHandler()); err != nil {
		setupLog.Error(err, "problem running manager")
		os.Exit(1)
	}
}
