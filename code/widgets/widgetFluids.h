#ifndef WIDGETFLUIDS_H
#define WIDGETFLUIDS_H

#include <QWidget>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>

class WidgetFluids : public QWidget
{
    Q_OBJECT

public:
    explicit WidgetFluids(QWidget* parent = nullptr);
    ~WidgetFluids() = default;

signals:
    void updatedParameters();

public:
    double getViscosity() const;
    double getGravity() const;
    bool showParticles() const;

private slots:
    void onParameterChanged();

private:
    QDoubleSpinBox* spinViscosity = nullptr;
    QDoubleSpinBox* spinGravity = nullptr;
    QCheckBox* checkShowParticles = nullptr;
};

#endif // WIDGETFLUIDS_H
