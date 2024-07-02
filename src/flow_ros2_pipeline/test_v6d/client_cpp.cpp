#include <memory>
#include <string>
#include <thread>

#include "arrow/api.h"
#include "arrow/io/api.h"

#include "vineyard/basic/ds/array.h"
#include "vineyard/client/client.h"
#include "vineyard/client/ds/object_meta.h"
#include "vineyard/common/util/logging.h"

int main() {

    std::shared_ptr<vineyard::Client> client = std::make_shared<vineyard::Client>();
    VINEYARD_CHECK_OK(client->Connect("/var/run/vineyard.sock"));
    LOG(INFO) << "Connected to IPCServer: " << "/var/run/vineyard.sock";

    std::vector<double> double_array = {1.0, 7.0, 3.0, 4.0, 2.0};
    vineyard::ArrayBuilder<double> builder(*client, double_array);
    auto sealed_double_array =
        std::dynamic_pointer_cast<vineyard::Array<double>>(builder.Seal(*client));

    vineyard::ObjectID id = sealed_double_array->id();
    LOG(INFO) << "successfully sealed, " << vineyard::ObjectIDToString(id) << " ...";

    CHECK(!sealed_double_array->IsPersist());
    CHECK(sealed_double_array->IsLocal());

    VINEYARD_CHECK_OK(sealed_double_array->Persist(*client));
    LOG(INFO) << "successfully persisted...";

    CHECK(sealed_double_array->IsPersist());
    CHECK(sealed_double_array->IsLocal());

    auto vy_double_array =
        std::dynamic_pointer_cast<vineyard::Array<double>>(client->GetObject(id));
    LOG(INFO) << "successfully obtained...";

    CHECK_EQ(sealed_double_array->size(), double_array.size());
    CHECK_EQ(vy_double_array->size(), double_array.size());
    for (size_t i = 0; i < double_array.size(); ++i) {
        CHECK_EQ((*sealed_double_array)[i], double_array[i]);
        CHECK_EQ((*vy_double_array)[i], double_array[i]);
    }

    LOG(INFO) << "Passed double array tests...";

    client->Disconnect();

    return 0;
}
