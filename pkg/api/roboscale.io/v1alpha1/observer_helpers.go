package v1alpha1

import (
	"k8s.io/apimachinery/pkg/types"
)

// ********************************
// MetricsExporter helpers
// ********************************

func (metricseexporter *MetricsExporter) GetMetricsExporterPodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: metricseexporter.Namespace,
		Name:      metricseexporter.Name,
	}
}

func (metricseexporter *MetricsExporter) GetMetricsExporterRoleMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: metricseexporter.Namespace,
		Name:      metricseexporter.Name + "-role",
	}
}

func (metricseexporter *MetricsExporter) GetMetricsExporterRoleBindingMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: metricseexporter.Namespace,
		Name:      metricseexporter.Name + "-rb",
	}
}

func (metricseexporter *MetricsExporter) GetMetricsExporterServiceAccountMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: metricseexporter.Namespace,
		Name:      metricseexporter.Name + "-sa",
	}
}
