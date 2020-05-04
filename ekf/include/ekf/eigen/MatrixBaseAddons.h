
bool isApproxSymmetric() const
{
    return this->isApprox(this->transpose(), 0.1);
}
