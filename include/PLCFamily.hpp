#ifndef PLCFAMILY_HPP
#define PLCFAMILY_HPP

#include <QObject>

class PLCFamily : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString plcFamilyDescription READ getPLCFamilyDescription)
    Q_PROPERTY(QString plcProgramName READ getPLCProgramName)
    Q_PROPERTY(QString plcFamilyId READ getPLCFamilyId)

public:
    explicit PLCFamily(QObject *parent = nullptr, QString plcFamilyDescription = "", QString plcFamilyId = "");
    ~PLCFamily();

    QString getPLCFamilyDescription() const;
    QString getPLCProgramName() const;
    QString getPLCFamilyId() const;

private:
    QString _plcFamilyDescription = "";
    QString _plcFamilyId = "";
};

#endif // PLCFAMILY_HPP
