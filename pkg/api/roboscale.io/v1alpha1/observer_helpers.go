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
