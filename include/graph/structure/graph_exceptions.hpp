#pragma once

#include <stdexcept>
#include <string>
#include <unordered_map>

#include <map>

namespace graph {
    enum GraphErrorCode {
        VERTEX_ALREADY_EXISTS,
        VERTEX_NOT_FOUND,
        EDGE_ALREADY_EXISTS,
        EDGE_NOT_FOUND,
        SELF_LOOPS_FORBIDDEN,
        OPERATION_NOT_SUPPORTED,
    };


    inline std::unordered_map<GraphErrorCode, std::string> error_messages = {
            {VERTEX_ALREADY_EXISTS, "Уже есть такая вершина!"},
            {VERTEX_NOT_FOUND, "Нет такой вершины!"},
            {EDGE_ALREADY_EXISTS, "Уже есть такое ребро!"},
            {EDGE_NOT_FOUND, "Нет такого ребра!"},
            {SELF_LOOPS_FORBIDDEN, "Петли не допускаются!"},
            {OPERATION_NOT_SUPPORTED, "Операция не поддерживается!"}};

    class GraphException : public std::runtime_error {
    public:
        GraphException(GraphErrorCode code, const std::string &message)
            : std::runtime_error(message), error_code_(code) {}

        GraphErrorCode getErrorCode() const { return error_code_; }

    private:
        GraphErrorCode error_code_;
    };

    inline GraphException vertexAlreadyExists() {
        return GraphException(GraphErrorCode::VERTEX_ALREADY_EXISTS, error_messages[GraphErrorCode::VERTEX_ALREADY_EXISTS]);
    }

    inline GraphException vertexNotFound() {
        return GraphException(GraphErrorCode::VERTEX_NOT_FOUND, error_messages[GraphErrorCode::VERTEX_NOT_FOUND]);
    }

    inline GraphException edgeAlreadyExists() {
        return GraphException(GraphErrorCode::EDGE_ALREADY_EXISTS, error_messages[GraphErrorCode::EDGE_ALREADY_EXISTS]);
    }

    inline GraphException edgeNotFound() {
        return GraphException(GraphErrorCode::EDGE_NOT_FOUND, error_messages[GraphErrorCode::EDGE_NOT_FOUND]);
    }

    inline GraphException selfLoopsForbidden() {
        return GraphException(GraphErrorCode::SELF_LOOPS_FORBIDDEN, error_messages[GraphErrorCode::SELF_LOOPS_FORBIDDEN]);
    }

    inline GraphException operationNotSupported() {
        return GraphException(GraphErrorCode::OPERATION_NOT_SUPPORTED, error_messages[GraphErrorCode::OPERATION_NOT_SUPPORTED]);
    }

}// namespace graph
