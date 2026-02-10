#ifndef PLCTAG_HPP
#define PLCTAG_HPP

#include <QObject>
#include <QHash>


class PLCTag : public QObject
{
    Q_OBJECT

public:
    explicit PLCTag(QObject *parent = nullptr);
    ~PLCTag();

signals:

public slots:

private:
    QHash<QString, int> _PLCTags;

    bool getPLCTag(QString tagName, int32_t &tag);
    bool readPLCTag(QString tagName, bool &tagValue);
    bool writePLCTag(QString tagName, bool tagValue);
};

#endif // PLCTAG_HPP
