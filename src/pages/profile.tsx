/**
 * Profile Page
 * Purpose: Render UserProfile component with mobile responsive layout
 * Date: 2025-12-24
 * Task: T144, T145
 */

import React from 'react';
import Layout from '@theme/Layout';
import { UserProfile } from '../components/auth/UserProfile';

export default function ProfilePage(): JSX.Element {
  return (
    <Layout
      title="My Profile"
      description="View and manage your profile information"
    >
      <UserProfile />
    </Layout>
  );
}
