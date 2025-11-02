#include "widgetfluids.h"

WidgetFluids::WidgetFluids(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);

    auto* lblTitle = new QLabel("<b>Fluid Simulation Parameters</b>");
    layout->addWidget(lblTitle);

    spinViscosity = new QDoubleSpinBox();
    spinViscosity->setRange(0.0, 100.0);
    spinViscosity->setSingleStep(0.1);
    spinViscosity->setValue(1.0);
    spinViscosity->setPrefix("Viscosity: ");
    layout->addWidget(spinViscosity);

    spinGravity = new QDoubleSpinBox();
    spinGravity->setRange(0.0, 50.0);
    spinGravity->setSingleStep(0.1);
    spinGravity->setValue(9.81);
    spinGravity->setPrefix("Gravity: ");
    layout->addWidget(spinGravity);

    checkShowParticles = new QCheckBox("Show Particles");
    checkShowParticles->setChecked(true);
    layout->addWidget(checkShowParticles);

    layout->addStretch();

    // Connect all parameter changes
    connect(spinViscosity, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &WidgetFluids::onParameterChanged);
    connect(spinGravity, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &WidgetFluids::onParameterChanged);
    connect(checkShowParticles, &QCheckBox::toggled,
            this, &WidgetFluids::onParameterChanged);
}

void WidgetFluids::onParameterChanged() {
    emit updatedParameters();
}

double WidgetFluids::getViscosity() const {
    return spinViscosity->value();
}

double WidgetFluids::getGravity() const {
    return spinGravity->value();
}

bool WidgetFluids::showParticles() const {
    return checkShowParticles->isChecked();
}
