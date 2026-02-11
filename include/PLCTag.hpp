#ifndef PLCTAG_HPP
#define PLCTAG_HPP

#include <QObject>
#include <QHash>


class PLCTag : public QObject
{
    Q_OBJECT
public:
    explicit PLCTag(QObject *parent = nullptr, QString plcAddress = "", QString plcType = "");
    ~PLCTag();

    int32_t getPLCTag(QString tagName);
    bool readPLCTag(QString tagName, bool &tagValue);
    bool writePLCTag(QString tagName, bool tagValue);

signals:

public slots:

private:
    QString _plcAddress = "";
    QString _plcType = "";
    int32_t _dataTimeout = 5000;
    QHash<QString, int32_t> _PLCTags;

};

#endif // PLCTAG_HPP
